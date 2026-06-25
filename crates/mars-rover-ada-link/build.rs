use std::path::PathBuf;
use std::process::{Command, Stdio};

const LIBRARY_NAME: &str = "Mars_Rover";
const LIBRARY_DIR: &str = "lib";

fn main() {
    // Resolve the Ada project directory relative to this crate's manifest,
    // then convert to absolute so rustc's -L flag is unambiguous regardless
    // of where cargo was invoked within the workspace.
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let project_dir = manifest_dir
        .join("../../simulator_interface")
        .canonicalize()
        .expect("simulator_interface directory not found");

    let output = Command::new("alr")
        .args(["build"])
        .current_dir(&project_dir)
        .stderr(Stdio::inherit())
        .output()
        .unwrap();

    if !output.status.success() {
        panic!();
    }

    println!("cargo::rerun-if-changed={}/mars_rover.gpr", project_dir.display());
    println!("cargo::rerun-if-changed={}/../src", project_dir.display());

    println!("cargo::rustc-link-search={}", project_dir.join(LIBRARY_DIR).display());
    println!("cargo::rustc-link-lib=static={LIBRARY_NAME}");

    let output = String::from_utf8(
        Command::new("alr")
            .args(["exec", "--", "gnatls", "-v"])
            .current_dir(&project_dir)
            .output()
            .expect("failed to execute gnatls")
            .stdout,
    )
    .expect("failed to convert output");

    let libgnat_dir = output
        .split('\n')
        .filter(|s| s.contains("adalib"))
        .map(|s| s.trim())
        .next()
        .expect("libgnat directory not found");

    println!("cargo::rustc-link-search={libgnat_dir}");
    println!("cargo::rustc-link-lib=static=gnat");

    if cfg!(target_os = "macos") {
        println!("cargo:rustc-env=DYLD_FALLBACK_LIBRARY_PATH={libgnat_dir}");
    }
}
