use std::process::{Command, Stdio};

const PROJECT_DIR: &str = "../simulator_interface";
const LIBRARY_DIR: &str = "lib";
const LIBRARY_NAME: &str = "Mars_Rover";

fn main() {
    let output = Command::new("alr")
        .args(["build"])
        .current_dir(PROJECT_DIR)
        .stderr(Stdio::inherit())
        .output()
        .unwrap();

    if !output.status.success() {
        panic!();
    }

    println!("cargo::rerun-if-changed={PROJECT_DIR}/mars_rover.gpr");
    println!("cargo::rerun-if-changed={PROJECT_DIR}/../src");

    println!("cargo::rustc-link-search={PROJECT_DIR}/{LIBRARY_DIR}");
    println!("cargo::rustc-link-lib=static={LIBRARY_NAME}");

    let output = String::from_utf8(
        Command::new("alr")
            .args(["exec", "--", "gnatls", "-v"])
            .current_dir(PROJECT_DIR)
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
}
