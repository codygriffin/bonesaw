//! Dynamically loaded NVRTC boundary for model-specialized CUDA stages.

#[cfg(target_os = "linux")]
mod platform {
    use std::{ffi::CString, os::raw::c_char, ptr};

    use thiserror::Error;

    type NvrtcProgram = *mut core::ffi::c_void;
    type NvrtcVersion = unsafe extern "C" fn(*mut i32, *mut i32) -> i32;
    type NvrtcCreateProgram = unsafe extern "C" fn(
        *mut NvrtcProgram,
        *const c_char,
        *const c_char,
        i32,
        *const *const c_char,
        *const *const c_char,
    ) -> i32;
    type NvrtcCompileProgram = unsafe extern "C" fn(NvrtcProgram, i32, *const *const c_char) -> i32;
    type NvrtcGetPtxSize = unsafe extern "C" fn(NvrtcProgram, *mut usize) -> i32;
    type NvrtcGetPtx = unsafe extern "C" fn(NvrtcProgram, *mut c_char) -> i32;
    type NvrtcGetProgramLogSize = unsafe extern "C" fn(NvrtcProgram, *mut usize) -> i32;
    type NvrtcGetProgramLog = unsafe extern "C" fn(NvrtcProgram, *mut c_char) -> i32;
    type NvrtcDestroyProgram = unsafe extern "C" fn(*mut NvrtcProgram) -> i32;

    unsafe extern "C" {
        fn dlopen(filename: *const c_char, flags: i32) -> *mut core::ffi::c_void;
        fn dlsym(handle: *mut core::ffi::c_void, symbol: *const c_char) -> *mut core::ffi::c_void;
        fn dlclose(handle: *mut core::ffi::c_void) -> i32;
    }

    const RTLD_NOW: i32 = 2;
    const LIBRARIES: [&str; 5] = [
        "libnvrtc.so.13",
        "libnvrtc.so.12",
        "libnvrtc.so.11.2",
        "libnvrtc.so.11.0",
        "libnvrtc.so",
    ];

    #[derive(Clone, Debug, Error, Eq, PartialEq)]
    pub enum NvrtcError {
        #[error("NVRTC library is unavailable")]
        LibraryUnavailable,
        #[error("NVRTC symbol {0} is unavailable")]
        MissingSymbol(&'static str),
        #[error("NVRTC input contains an interior NUL")]
        InputNul,
        #[error("NVRTC operation {operation} failed with result {code}: {log}")]
        Call {
            operation: &'static str,
            code: i32,
            log: String,
        },
        #[error("NVRTC returned non-UTF8 PTX")]
        NonUtf8Ptx,
    }

    #[derive(Debug)]
    struct Library(*mut core::ffi::c_void);

    impl Library {
        fn open() -> Result<Self, NvrtcError> {
            for name in LIBRARIES {
                let name = CString::new(name).expect("static NVRTC library name");
                // SAFETY: name is NUL-terminated.
                let handle = unsafe { dlopen(name.as_ptr(), RTLD_NOW) };
                if !handle.is_null() {
                    return Ok(Self(handle));
                }
            }
            Err(NvrtcError::LibraryUnavailable)
        }

        unsafe fn symbol<T: Copy>(&self, name: &'static str) -> Result<T, NvrtcError> {
            let symbol = CString::new(name).expect("static NVRTC symbol name");
            // SAFETY: the library is live and the symbol name is valid.
            let address = unsafe { dlsym(self.0, symbol.as_ptr()) };
            if address.is_null() {
                return Err(NvrtcError::MissingSymbol(name));
            }
            debug_assert_eq!(core::mem::size_of::<T>(), core::mem::size_of_val(&address));
            // SAFETY: caller supplies the corresponding NVRTC function type.
            Ok(unsafe { core::mem::transmute_copy(&address) })
        }
    }

    impl Drop for Library {
        fn drop(&mut self) {
            if !self.0.is_null() {
                // SAFETY: this object owns the handle.
                let _ = unsafe { dlclose(self.0) };
            }
        }
    }

    #[derive(Debug)]
    pub struct NvrtcCompiler {
        _library: Library,
        version: NvrtcVersion,
        create_program: NvrtcCreateProgram,
        compile_program: NvrtcCompileProgram,
        get_ptx_size: NvrtcGetPtxSize,
        get_ptx: NvrtcGetPtx,
        get_log_size: NvrtcGetProgramLogSize,
        get_log: NvrtcGetProgramLog,
        destroy_program: NvrtcDestroyProgram,
    }

    impl NvrtcCompiler {
        pub fn load() -> Result<Self, NvrtcError> {
            let library = Library::open()?;
            // SAFETY: names and function types match the public NVRTC ABI.
            unsafe {
                Ok(Self {
                    version: library.symbol("nvrtcVersion")?,
                    create_program: library.symbol("nvrtcCreateProgram")?,
                    compile_program: library.symbol("nvrtcCompileProgram")?,
                    get_ptx_size: library.symbol("nvrtcGetPTXSize")?,
                    get_ptx: library.symbol("nvrtcGetPTX")?,
                    get_log_size: library.symbol("nvrtcGetProgramLogSize")?,
                    get_log: library.symbol("nvrtcGetProgramLog")?,
                    destroy_program: library.symbol("nvrtcDestroyProgram")?,
                    _library: library,
                })
            }
        }

        pub fn version(&self) -> Result<(i32, i32), NvrtcError> {
            let mut major = 0;
            let mut minor = 0;
            // SAFETY: both output pointers are valid.
            let code = unsafe { (self.version)(&mut major, &mut minor) };
            check("nvrtcVersion", code, String::new())?;
            Ok((major, minor))
        }

        pub fn compile(
            &self,
            source: &str,
            name: &str,
            compute_major: i32,
            compute_minor: i32,
        ) -> Result<(String, String), NvrtcError> {
            let source = CString::new(source).map_err(|_| NvrtcError::InputNul)?;
            let name = CString::new(name).map_err(|_| NvrtcError::InputNul)?;
            let architecture = CString::new(format!(
                "--gpu-architecture=compute_{compute_major}{compute_minor}"
            ))
            .map_err(|_| NvrtcError::InputNul)?;
            let option_values = [
                architecture,
                CString::new("--fmad=true").unwrap(),
                CString::new("--ftz=false").unwrap(),
                CString::new("--prec-div=true").unwrap(),
                CString::new("--prec-sqrt=true").unwrap(),
                CString::new("--std=c++14").unwrap(),
            ];
            let options = option_values
                .iter()
                .map(|value| value.as_ptr())
                .collect::<Vec<_>>();
            let mut program = ptr::null_mut();
            // SAFETY: source/name live through program creation; there are no headers.
            check(
                "nvrtcCreateProgram",
                unsafe {
                    (self.create_program)(
                        &mut program,
                        source.as_ptr(),
                        name.as_ptr(),
                        0,
                        ptr::null(),
                        ptr::null(),
                    )
                },
                String::new(),
            )?;
            let compile_code =
                unsafe { (self.compile_program)(program, options.len() as i32, options.as_ptr()) };
            let log = self.program_log(program);
            if compile_code != 0 {
                // SAFETY: program was created successfully and is uniquely owned here.
                let _ = unsafe { (self.destroy_program)(&mut program) };
                return Err(NvrtcError::Call {
                    operation: "nvrtcCompileProgram",
                    code: compile_code,
                    log,
                });
            }
            let mut size = 0;
            let size_code = unsafe { (self.get_ptx_size)(program, &mut size) };
            if size_code != 0 {
                let _ = unsafe { (self.destroy_program)(&mut program) };
                return Err(NvrtcError::Call {
                    operation: "nvrtcGetPTXSize",
                    code: size_code,
                    log,
                });
            }
            let mut bytes = vec![0_u8; size];
            let ptx_code = unsafe { (self.get_ptx)(program, bytes.as_mut_ptr().cast()) };
            let _ = unsafe { (self.destroy_program)(&mut program) };
            check("nvrtcGetPTX", ptx_code, log.clone())?;
            if bytes.last() == Some(&0) {
                bytes.pop();
            }
            let ptx = String::from_utf8(bytes).map_err(|_| NvrtcError::NonUtf8Ptx)?;
            Ok((ptx, log))
        }

        fn program_log(&self, program: NvrtcProgram) -> String {
            let mut size = 0;
            // SAFETY: program is live and size is writable.
            if unsafe { (self.get_log_size)(program, &mut size) } != 0 || size == 0 {
                return String::new();
            }
            let mut bytes = vec![0_u8; size];
            // SAFETY: buffer is sized from NVRTC's query.
            if unsafe { (self.get_log)(program, bytes.as_mut_ptr().cast()) } != 0 {
                return String::new();
            }
            if bytes.last() == Some(&0) {
                bytes.pop();
            }
            String::from_utf8_lossy(&bytes).into_owned()
        }
    }

    fn check(operation: &'static str, code: i32, log: String) -> Result<(), NvrtcError> {
        if code == 0 {
            Ok(())
        } else {
            Err(NvrtcError::Call {
                operation,
                code,
                log,
            })
        }
    }
}

#[cfg(not(target_os = "linux"))]
mod platform {
    use thiserror::Error;

    #[derive(Clone, Debug, Error, Eq, PartialEq)]
    pub enum NvrtcError {
        #[error("dynamic NVRTC loading is currently supported only on Linux")]
        UnsupportedPlatform,
    }

    #[derive(Debug)]
    pub struct NvrtcCompiler;

    impl NvrtcCompiler {
        pub fn load() -> Result<Self, NvrtcError> {
            Err(NvrtcError::UnsupportedPlatform)
        }
    }
}

pub use platform::*;
