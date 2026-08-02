//! Minimal dynamically loaded CUDA Driver API boundary.
//!
//! Keeping this private avoids a link-time dependency on `libcuda` and lets a
//! CPU-only deployment report typed unavailability instead of failing to load
//! the Bonesaw library. No CUDA symbol is resolved on a control-path call;
//! executor construction owns all discovery.

#[cfg(target_os = "linux")]
mod platform {
    use std::{ffi::CString, os::raw::c_char, ptr};

    use thiserror::Error;

    pub type CuDevice = i32;
    pub type CuDevicePtr = u64;
    pub type CuContext = *mut core::ffi::c_void;
    pub type CuModule = *mut core::ffi::c_void;
    pub type CuFunction = *mut core::ffi::c_void;
    pub type CuStream = *mut core::ffi::c_void;

    pub const CUDA_ERROR_NO_DEVICE: i32 = 100;
    pub const CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT: i32 = 16;
    pub const CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR: i32 = 75;
    pub const CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR: i32 = 76;

    type CuInit = unsafe extern "C" fn(u32) -> i32;
    type CuDriverGetVersion = unsafe extern "C" fn(*mut i32) -> i32;
    type CuDeviceGetCount = unsafe extern "C" fn(*mut i32) -> i32;
    type CuDeviceGet = unsafe extern "C" fn(*mut CuDevice, i32) -> i32;
    type CuDeviceGetName = unsafe extern "C" fn(*mut c_char, i32, CuDevice) -> i32;
    type CuDeviceGetAttribute = unsafe extern "C" fn(*mut i32, i32, CuDevice) -> i32;
    type CuCtxCreate = unsafe extern "C" fn(*mut CuContext, u32, CuDevice) -> i32;
    type CuCtxDestroy = unsafe extern "C" fn(CuContext) -> i32;
    type CuCtxSetCurrent = unsafe extern "C" fn(CuContext) -> i32;
    type CuCtxSynchronize = unsafe extern "C" fn() -> i32;
    type CuModuleLoadData = unsafe extern "C" fn(*mut CuModule, *const core::ffi::c_void) -> i32;
    type CuModuleUnload = unsafe extern "C" fn(CuModule) -> i32;
    type CuModuleGetFunction =
        unsafe extern "C" fn(*mut CuFunction, CuModule, *const c_char) -> i32;
    type CuMemAlloc = unsafe extern "C" fn(*mut CuDevicePtr, usize) -> i32;
    type CuMemFree = unsafe extern "C" fn(CuDevicePtr) -> i32;
    type CuMemcpyHtoD = unsafe extern "C" fn(CuDevicePtr, *const core::ffi::c_void, usize) -> i32;
    type CuMemcpyDtoH = unsafe extern "C" fn(*mut core::ffi::c_void, CuDevicePtr, usize) -> i32;
    type CuLaunchKernel = unsafe extern "C" fn(
        CuFunction,
        u32,
        u32,
        u32,
        u32,
        u32,
        u32,
        u32,
        CuStream,
        *mut *mut core::ffi::c_void,
        *mut *mut core::ffi::c_void,
    ) -> i32;

    unsafe extern "C" {
        fn dlopen(filename: *const c_char, flags: i32) -> *mut core::ffi::c_void;
        fn dlsym(handle: *mut core::ffi::c_void, symbol: *const c_char) -> *mut core::ffi::c_void;
        fn dlclose(handle: *mut core::ffi::c_void) -> i32;
    }

    const RTLD_NOW: i32 = 2;
    const CUDA_LIBRARIES: [&str; 2] = ["libcuda.so.1", "libcuda.so"];

    #[derive(Clone, Debug, Error, Eq, PartialEq)]
    pub enum DriverError {
        #[error("CUDA driver library is unavailable")]
        LibraryUnavailable,
        #[error("CUDA driver symbol {0} is unavailable")]
        MissingSymbol(&'static str),
        #[error("CUDA operation {operation} failed with CUresult {code}")]
        Call { operation: &'static str, code: i32 },
        #[error("CUDA device ordinal {ordinal} is unavailable; device count is {count}")]
        DeviceOrdinal { ordinal: i32, count: i32 },
        #[error("CUDA kernel text contains an interior NUL")]
        KernelNul,
    }

    pub fn check(operation: &'static str, code: i32) -> Result<(), DriverError> {
        if code == 0 {
            Ok(())
        } else {
            Err(DriverError::Call { operation, code })
        }
    }

    #[derive(Debug)]
    struct DynamicLibrary(*mut core::ffi::c_void);

    impl DynamicLibrary {
        fn open() -> Result<Self, DriverError> {
            for name in CUDA_LIBRARIES {
                let name = CString::new(name).expect("static CUDA library name");
                // SAFETY: `name` is NUL-terminated and dlopen owns no Rust data.
                let handle = unsafe { dlopen(name.as_ptr(), RTLD_NOW) };
                if !handle.is_null() {
                    return Ok(Self(handle));
                }
            }
            Err(DriverError::LibraryUnavailable)
        }

        unsafe fn symbol<T: Copy>(&self, name: &'static str) -> Result<T, DriverError> {
            let symbol = CString::new(name).expect("static CUDA symbol name");
            // SAFETY: the library handle is live and the symbol name is NUL-terminated.
            let address = unsafe { dlsym(self.0, symbol.as_ptr()) };
            if address.is_null() {
                return Err(DriverError::MissingSymbol(name));
            }
            debug_assert_eq!(
                core::mem::size_of::<T>(),
                core::mem::size_of::<*mut core::ffi::c_void>()
            );
            // SAFETY: each caller supplies the exact CUDA function-pointer type.
            Ok(unsafe { core::mem::transmute_copy(&address) })
        }
    }

    impl Drop for DynamicLibrary {
        fn drop(&mut self) {
            if !self.0.is_null() {
                // SAFETY: this object uniquely owns the live dlopen handle.
                let _ = unsafe { dlclose(self.0) };
            }
        }
    }

    #[derive(Debug)]
    pub struct Driver {
        _library: DynamicLibrary,
        cu_init: CuInit,
        cu_driver_get_version: CuDriverGetVersion,
        cu_device_get_count: CuDeviceGetCount,
        cu_device_get: CuDeviceGet,
        cu_device_get_name: CuDeviceGetName,
        cu_device_get_attribute: CuDeviceGetAttribute,
        cu_ctx_create: CuCtxCreate,
        cu_ctx_destroy: CuCtxDestroy,
        cu_ctx_set_current: CuCtxSetCurrent,
        cu_ctx_synchronize: CuCtxSynchronize,
        cu_module_load_data: CuModuleLoadData,
        cu_module_unload: CuModuleUnload,
        cu_module_get_function: CuModuleGetFunction,
        cu_mem_alloc: CuMemAlloc,
        cu_mem_free: CuMemFree,
        cu_memcpy_htod: CuMemcpyHtoD,
        cu_memcpy_dtoh: CuMemcpyDtoH,
        cu_launch_kernel: CuLaunchKernel,
    }

    impl Driver {
        pub fn load() -> Result<Self, DriverError> {
            let library = DynamicLibrary::open()?;
            // SAFETY: each lookup names a symbol from the CUDA Driver API with
            // the corresponding function-pointer type above.
            unsafe {
                Ok(Self {
                    cu_init: library.symbol("cuInit")?,
                    cu_driver_get_version: library.symbol("cuDriverGetVersion")?,
                    cu_device_get_count: library.symbol("cuDeviceGetCount")?,
                    cu_device_get: library.symbol("cuDeviceGet")?,
                    cu_device_get_name: library.symbol("cuDeviceGetName")?,
                    cu_device_get_attribute: library.symbol("cuDeviceGetAttribute")?,
                    cu_ctx_create: library.symbol("cuCtxCreate_v2")?,
                    cu_ctx_destroy: library.symbol("cuCtxDestroy_v2")?,
                    cu_ctx_set_current: library.symbol("cuCtxSetCurrent")?,
                    cu_ctx_synchronize: library.symbol("cuCtxSynchronize")?,
                    cu_module_load_data: library.symbol("cuModuleLoadData")?,
                    cu_module_unload: library.symbol("cuModuleUnload")?,
                    cu_module_get_function: library.symbol("cuModuleGetFunction")?,
                    cu_mem_alloc: library.symbol("cuMemAlloc_v2")?,
                    cu_mem_free: library.symbol("cuMemFree_v2")?,
                    cu_memcpy_htod: library.symbol("cuMemcpyHtoD_v2")?,
                    cu_memcpy_dtoh: library.symbol("cuMemcpyDtoH_v2")?,
                    cu_launch_kernel: library.symbol("cuLaunchKernel")?,
                    _library: library,
                })
            }
        }

        pub fn initialize(&self) -> Result<(), DriverError> {
            // SAFETY: CUDA initialization takes no pointers and is process-scoped.
            check("cuInit", unsafe { (self.cu_init)(0) })
        }

        pub fn driver_version(&self) -> Result<i32, DriverError> {
            let mut version = 0;
            // SAFETY: `version` is a valid writable i32.
            check("cuDriverGetVersion", unsafe {
                (self.cu_driver_get_version)(&mut version)
            })?;
            Ok(version)
        }

        pub fn device_count(&self) -> Result<i32, DriverError> {
            let mut count = 0;
            // SAFETY: `count` is a valid writable i32.
            check("cuDeviceGetCount", unsafe {
                (self.cu_device_get_count)(&mut count)
            })?;
            Ok(count)
        }

        pub fn device(&self, ordinal: i32) -> Result<CuDevice, DriverError> {
            let count = self.device_count()?;
            if ordinal < 0 || ordinal >= count {
                return Err(DriverError::DeviceOrdinal { ordinal, count });
            }
            let mut device = 0;
            // SAFETY: `device` is writable and the ordinal was range-checked.
            check("cuDeviceGet", unsafe {
                (self.cu_device_get)(&mut device, ordinal)
            })?;
            Ok(device)
        }

        pub fn device_name(&self, device: CuDevice) -> Result<String, DriverError> {
            let mut bytes = [0_i8; 256];
            // SAFETY: the fixed buffer is writable for the supplied length.
            check("cuDeviceGetName", unsafe {
                (self.cu_device_get_name)(bytes.as_mut_ptr(), bytes.len() as i32, device)
            })?;
            let length = bytes
                .iter()
                .position(|value| *value == 0)
                .unwrap_or(bytes.len());
            let unsigned = bytes[..length]
                .iter()
                .map(|value| *value as u8)
                .collect::<Vec<_>>();
            Ok(String::from_utf8_lossy(&unsigned).into_owned())
        }

        pub fn device_attribute(
            &self,
            device: CuDevice,
            attribute: i32,
        ) -> Result<i32, DriverError> {
            let mut value = 0;
            // SAFETY: `value` is writable and the CUDA driver validates the attribute.
            check("cuDeviceGetAttribute", unsafe {
                (self.cu_device_get_attribute)(&mut value, attribute, device)
            })?;
            Ok(value)
        }

        pub fn create_context(&self, device: CuDevice) -> Result<CuContext, DriverError> {
            let mut context = ptr::null_mut();
            // SAFETY: `context` is writable and `device` came from the driver.
            check("cuCtxCreate_v2", unsafe {
                (self.cu_ctx_create)(&mut context, 0, device)
            })?;
            Ok(context)
        }

        pub fn destroy_context(&self, context: CuContext) {
            if !context.is_null() {
                // SAFETY: the executor uniquely owns this context.
                let _ = unsafe { (self.cu_ctx_destroy)(context) };
            }
        }

        pub fn set_current(&self, context: CuContext) -> Result<(), DriverError> {
            // SAFETY: the executor retains the context for this call.
            check("cuCtxSetCurrent", unsafe {
                (self.cu_ctx_set_current)(context)
            })
        }

        pub fn synchronize(&self) -> Result<(), DriverError> {
            // SAFETY: synchronizes the current context only.
            check("cuCtxSynchronize", unsafe { (self.cu_ctx_synchronize)() })
        }

        pub fn load_module(&self, ptx: &str) -> Result<CuModule, DriverError> {
            let ptx = CString::new(ptx).map_err(|_| DriverError::KernelNul)?;
            let mut module = ptr::null_mut();
            // SAFETY: the PTX is NUL-terminated and remains live during JIT loading.
            check("cuModuleLoadData", unsafe {
                (self.cu_module_load_data)(&mut module, ptx.as_ptr().cast::<core::ffi::c_void>())
            })?;
            Ok(module)
        }

        pub fn unload_module(&self, module: CuModule) {
            if !module.is_null() {
                // SAFETY: the executor uniquely owns this module.
                let _ = unsafe { (self.cu_module_unload)(module) };
            }
        }

        pub fn function(
            &self,
            module: CuModule,
            name: &'static str,
        ) -> Result<CuFunction, DriverError> {
            let name = CString::new(name).expect("static CUDA function name");
            let mut function = ptr::null_mut();
            // SAFETY: the module is live and the name is NUL-terminated.
            check("cuModuleGetFunction", unsafe {
                (self.cu_module_get_function)(&mut function, module, name.as_ptr())
            })?;
            Ok(function)
        }

        pub fn allocate(&self, bytes: usize) -> Result<CuDevicePtr, DriverError> {
            let mut pointer = 0;
            // SAFETY: `pointer` is writable; the driver owns the allocation.
            check("cuMemAlloc_v2", unsafe {
                (self.cu_mem_alloc)(&mut pointer, bytes)
            })?;
            Ok(pointer)
        }

        pub fn free(&self, pointer: CuDevicePtr) {
            if pointer != 0 {
                // SAFETY: the executor uniquely owns this allocation.
                let _ = unsafe { (self.cu_mem_free)(pointer) };
            }
        }

        pub fn copy_to_device<T>(
            &self,
            destination: CuDevicePtr,
            source: &[T],
        ) -> Result<(), DriverError> {
            let bytes = core::mem::size_of_val(source);
            // SAFETY: source is readable for `bytes`; destination was sized identically.
            check("cuMemcpyHtoD_v2", unsafe {
                (self.cu_memcpy_htod)(destination, source.as_ptr().cast(), bytes)
            })
        }

        pub fn copy_from_device<T>(
            &self,
            destination: &mut [T],
            source: CuDevicePtr,
        ) -> Result<(), DriverError> {
            let bytes = core::mem::size_of_val(destination);
            // SAFETY: destination is writable for `bytes`; source was sized identically.
            check("cuMemcpyDtoH_v2", unsafe {
                (self.cu_memcpy_dtoh)(destination.as_mut_ptr().cast(), source, bytes)
            })
        }

        pub fn launch(
            &self,
            function: CuFunction,
            grid_x: u32,
            block_x: u32,
            parameters: &mut [*mut core::ffi::c_void],
        ) -> Result<(), DriverError> {
            // SAFETY: parameters point to live kernel argument storage for the
            // synchronous launch; the default stream belongs to this context.
            check("cuLaunchKernel", unsafe {
                (self.cu_launch_kernel)(
                    function,
                    grid_x,
                    1,
                    1,
                    block_x,
                    1,
                    1,
                    0,
                    ptr::null_mut(),
                    parameters.as_mut_ptr(),
                    ptr::null_mut(),
                )
            })
        }
    }
}

#[cfg(not(target_os = "linux"))]
mod platform {
    use thiserror::Error;

    pub const CUDA_ERROR_NO_DEVICE: i32 = 100;

    #[derive(Clone, Debug, Error, Eq, PartialEq)]
    pub enum DriverError {
        #[error("dynamic CUDA driver loading is currently supported only on Linux")]
        UnsupportedPlatform,
    }

    #[derive(Debug)]
    pub struct Driver;

    impl Driver {
        pub fn load() -> Result<Self, DriverError> {
            Err(DriverError::UnsupportedPlatform)
        }
    }
}

pub use platform::*;
