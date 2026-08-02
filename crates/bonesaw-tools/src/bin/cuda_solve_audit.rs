#[path = "cpu_mirror_solve_audit.rs"]
mod cpu_mirror_solve_audit;

fn main() -> anyhow::Result<()> {
    cpu_mirror_solve_audit::run(true)
}
