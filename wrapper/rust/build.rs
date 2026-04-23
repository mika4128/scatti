use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let scatti_root = PathBuf::from(&manifest_dir).join("..").join("..");
    let scatti_root = scatti_root.canonicalize().expect("cannot resolve scatti root");

    let include_dir = scatti_root.join("include");
    let src_dir = scatti_root.join("src");

    let h = include_dir.join("scatti");
    println!("cargo:rerun-if-changed={}", h.join("scatti.h").display());
    println!("cargo:rerun-if-changed={}", h.join("input_parameter.h").display());
    println!("cargo:rerun-if-changed={}", h.join("output_parameter.h").display());
    println!("cargo:rerun-if-changed={}", h.join("trajectory.h").display());

    // Compile scatti C sources directly into the Rust crate
    let sources: Vec<PathBuf> = [
        "roots.c",
        "brake.c",
        "profile.c",
        "block.c",
        "position_first_step1.c",
        "position_first_step2.c",
        "position_second_step1.c",
        "position_second_step2.c",
        "position_third_step1.c",
        "position_third_step2.c",
        "velocity_second_step1.c",
        "velocity_second_step2.c",
        "velocity_third_step1.c",
        "velocity_third_step2.c",
        "trajectory.c",
        "calculator.c",
        "input_parameter.c",
        "output_parameter.c",
        "scatti.c",
    ]
    .iter()
    .map(|f| src_dir.join(f))
    .collect();

    cc::Build::new()
        .std("c99")
        .files(&sources)
        .include(&include_dir)
        .flag("-O2")
        .flag("-DNDEBUG")
        .flag("-DSCATTI_OPT_LEVEL=0")
        .compile("scatti");

    println!("cargo:rustc-link-lib=static=scatti");
    println!("cargo:rustc-link-lib=m");

    // Generate bindings from the umbrella header
    // Only allowlist functions; pulling in `allowlist_type("CRuckig.*")` was emitting a
    // truncated `CRuckigInputParameter` (missing Pro fields) on some bindgen/libclang combos.
    let bindings = bindgen::Builder::default()
        .header(include_dir.join("scatti").join("scatti.h").to_str().unwrap())
        .clang_arg(format!("-I{}", include_dir.display()))
        .allowlist_function("scatti_.*")
        .allowlist_var("PROFILE_.*")
        .derive_debug(true)
        .derive_default(true)
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings");
}
