// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

//! This module is capable of generating C code for generating a binary X.509
//! certificate according to a [`Template`].

use anyhow::{Context, Result};
use heck::ToUpperCamelCase;
use indexmap::IndexMap;
use itertools::Itertools;
use std::fmt::Write;

use crate::asn1::codegen::{self, CodegenOutput, VariableCodegenInfo, VariableInfo};
use crate::asn1::x509::X509;
use crate::template::subst::{Subst, SubstValue};
use crate::template::{
    EcdsaSignature, Signature, SizeRange, Template, Value, Variable, VariableType,
};
use crate::x509;

/// The amount of test cases to generate for covering more corner cases.
const TEST_CASE_COUNT: u32 = 100;

pub struct Codegen {
    /// Header.
    pub source_h: String,
    /// Code containing the template and setters.
    pub source_c: String,
    /// Code containing the unittest.
    pub source_unittest: String,
}

/// Generate the certificate template header and source file.
///
/// The generated files will indicate that they have been automatically
/// generated from `from_file`.
/// Returns the implementation first and the header file second.
/// NOTE: the implementation file will `#include "<header>.h"` the header
/// where `<header>` comes from `tmpl.name`.
///
/// The generated header file contains the following elements. Below `<name>`
/// refers to `tmpl.name` and `<Name>` to the "camel-case" variant of `<name>`.
/// 1. License header, warning, include guard.
/// 2. Relevant includes.
/// 3. Definition of a data structure to hold the values of the variables
///    used in the TBS. It is named `<name>_tbs_values_t`.
/// 4. Definition of a data structure to hold the values of the variables
///    used in the signature. It is named `<name>_sig_values_t`. Note that this
///    structure contains an extra field called `tbs` (and its size `tbs_size`)
///    that must point to the buffer containing the TBS.
/// 5. An enumeration hold two values: one gives the maximum size of the TBS
///    given the variables sizes defined in the template, and another one for
///    the maximum size of the whole certificate. They are named
///    `k<Name>MaxTbsSizeBytes` and `k<Name>MaxCertSizeBytes` respectively.
/// 6. Definition and documentation of a function that takes as input a
///    a `<name>_tbs_values_t` and a buffer to produce the TBS. It is named
///    `<name>_build_tbs` and returns a `rom_error_t`.
/// 7. Definition and documentation of a function that takes as input a
///    a `<name>_sig_values_t` and a buffer to produce the certificate.
///    It is named `<name>_build_cert` and returns a `rom_error_t`.
pub fn generate_cert(from_file: &str, tmpl: &Template) -> Result<Codegen> {
    let mut source_c = String::new();
    let mut source_h = String::new();
    let mut source_unittest = String::new();

    let license_and_warning = indoc::formatdoc! { r#"
    // Copyright lowRISC contributors (OpenTitan project).
    // Licensed under the Apache License, Version 2.0, see LICENSE for details.
    // SPDX-License-Identifier: Apache-2.0

    // This file was automatically generated using opentitantool from:
    // {from_file}
    "#};

    // License, warning about autogenerated code and guard inclusion checks.
    source_c.push_str(&license_and_warning);
    source_h.push_str(&license_and_warning);
    let preproc_guard_include = tmpl.name.to_uppercase();
    writeln!(source_h, "#ifndef __{}__", preproc_guard_include)?;
    writeln!(source_h, "#define __{}__\n", preproc_guard_include)?;

    // Headers inclusion.
    source_c.push('\n');
    writeln!(source_c, "#include \"{}.h\"", tmpl.name)?;
    source_c.push_str("#include \"sw/device/silicon_creator/lib/cert/asn1.h\"\n\n");
    source_c.push_str("#include \"sw/device/silicon_creator/lib/cert/template.h\"\n\n");

    source_h.push_str("#include \"sw/device/lib/base/status.h\"\n\n");

    // Partition variables between TBS and signature.
    let mut tbs_vars = IndexMap::<String, VariableType>::new();
    let mut sig_vars = IndexMap::<String, VariableType>::new();
    for (var_name, var) in tmpl.variables.clone() {
        if var_appears_in_sig(&var_name, &tmpl.certificate.signature) {
            sig_vars.insert(var_name, var);
        } else {
            tbs_vars.insert(var_name, var);
        }
    }

    // Structure containing the TBS variables.
    let tbs_value_struct_name = format!("{}_tbs_values", tmpl.name);
    source_h.push_str(&generate_value_struct(&tbs_value_struct_name, &tbs_vars));
    let tbs_value_struct_name = tbs_value_struct_name + "_t";

    // Generate TBS function.
    let generate_tbs_fn_name = format!("{}_build_tbs", tmpl.name);
    let generate_tbs_fn_params =
        format!("{tbs_value_struct_name} *values, uint8_t *out_buf, size_t *inout_size");
    let (generate_tbs_fn_def, generate_tbs_fn_impl) = generate_builder(
        CertificateComponent::Tbs,
        &generate_tbs_fn_name,
        &generate_tbs_fn_params,
        &tbs_vars,
        |builder| X509::push_tbs_certificate(builder, &tmpl.certificate),
    )?;

    // Create a special variable to hold the TBS binary.
    let tbs_binary_val_name = "tbs";
    sig_vars.insert(
        tbs_binary_val_name.to_string(),
        VariableType::ByteArray {
            size: SizeRange::RangeSize(
                generate_tbs_fn_impl.min_size,
                generate_tbs_fn_impl.max_size,
            ),
            tweak_msb: None,
        },
    );
    let tbs_binary_val = Value::Variable(Variable {
        name: tbs_binary_val_name.to_string(),
        convert: None,
    });

    // Structure containing the signature variables.
    let sig_value_struct_name = format!("{}_sig_values", tmpl.name);
    source_h.push_str(&generate_value_struct(&sig_value_struct_name, &sig_vars));
    let sig_value_struct_name = sig_value_struct_name + "_t";

    // Generate sig function.
    let generate_cert_fn_name = format!("{}_build_cert", tmpl.name);
    let generate_cert_fn_params =
        format!("{sig_value_struct_name} *values, uint8_t *out_buf, size_t *inout_size");
    let (generate_cert_fn_def, generate_cert_fn_impl) = generate_builder(
        CertificateComponent::Certificate,
        &generate_cert_fn_name,
        &generate_cert_fn_params,
        &sig_vars,
        |builder| X509::push_certificate(builder, &tbs_binary_val, &tmpl.certificate.signature),
    )?;

    let tmpl_name = tmpl.name.to_upper_camel_case();

    // Create constants for the variable size range.
    source_h.push_str("enum {\n");
    for (var_name, var_type) in tbs_vars.iter().chain(sig_vars.iter()) {
        let (codegen, _) = c_variable_info(var_name, "", var_type);
        let const_name = var_name.to_upper_camel_case();
        let (min_size, max_size) = if let VariableCodegenInfo::Pointer { .. } = codegen {
            let (min_size, max_size) = var_type.array_size();
            if var_type.has_constant_array_size() {
                writeln!(
                    source_h,
                    "k{tmpl_name}Exact{const_name}SizeBytes = {max_size},"
                )?;
            }
            (min_size, max_size)
        } else {
            // Arbitrary range to hold scalars.
            (1, 100)
        };
        writeln!(
            source_h,
            "k{tmpl_name}Min{const_name}SizeBytes = {min_size},"
        )?;
        writeln!(
            source_h,
            "k{tmpl_name}Max{const_name}SizeBytes = {max_size},"
        )?;
    }
    source_h.push_str("};\n");

    // Create field name mapping.
    for (var_name, _) in tbs_vars.iter().chain(sig_vars.iter()) {
        let const_name = var_name.to_upper_camel_case();
        writeln!(source_h, "#define k{tmpl_name}Field{const_name} {var_name}")?;
    }

    let max_cert_size_const_name = format!("k{}MaxCertSizeBytes", tmpl.name.to_upper_camel_case());
    source_h.push_str("// Maximum possible size of a certificate\n");
    source_h.push_str(&indoc::formatdoc! {"enum {{
        {max_cert_size_const_name} = {},
    }};", generate_cert_fn_impl.max_size});

    // Output definition of the functions.
    source_h.push_str("\n\n");
    source_h.push_str(&generate_tbs_fn_def);
    source_h.push_str(&generate_cert_fn_def);
    source_h.push('\n');

    // Output the implementation of the functions.
    source_c.push_str(&generate_tbs_fn_impl.code);
    source_c.push_str(&generate_cert_fn_impl.code);
    source_c.push('\n');

    writeln!(source_h, "\n#endif /* __{}__ */", preproc_guard_include)?;

    // Generate unittest.
    source_unittest.push_str(&license_and_warning);
    source_unittest.push('\n');
    source_unittest.push_str("extern \"C\" {\n");
    writeln!(source_unittest, "#include \"{}.h\"", tmpl.name)?;
    source_unittest.push_str("}\n");
    source_unittest.push_str("#include \"gtest/gtest.h\"\n\n");

    for idx in 0..TEST_CASE_COUNT {
        let test_case = generate_test_case(
            &format!("Verify{idx}"),
            &tbs_vars,
            &sig_vars,
            generate_tbs_fn_impl.min_size,
            generate_tbs_fn_impl.max_size,
            generate_cert_fn_impl.max_size,
            tmpl,
        )?;
        source_unittest.push_str(&test_case);
    }

    Ok(Codegen {
        source_h,
        source_c,
        source_unittest,
    })
}

// Generate a unit test test case with random variables.
fn generate_test_case(
    test_name: &str,
    tbs_vars: &IndexMap<String, VariableType>,
    sig_vars: &IndexMap<String, VariableType>,
    min_tbs_size: usize,
    max_tbs_size: usize,
    max_cert_size: usize,
    tmpl: &Template,
) -> Result<String> {
    let mut source_unittest = String::new();
    let unittest_data = tmpl.random_test()?;
    let expected_cert = x509::generate_certificate(&tmpl.subst(&unittest_data)?)?;

    let tmpl_name = tmpl.name.to_upper_camel_case();
    let generate_tbs_fn_name = format!("{}_build_tbs", tmpl.name);
    let generate_cert_fn_name = format!("{}_build_cert", tmpl.name);
    let tbs_value_struct_name = format!("{}_tbs_values_t", tmpl.name);
    let sig_value_struct_name = format!("{}_sig_values_t", tmpl.name);

    source_unittest.push_str(&format! { r#"
        TEST({tmpl_name}, {test_name})
    "#});

    source_unittest.push_str(
        "
        {
    ",
    );

    // Generate constants holding the data.
    for (var_name, data) in unittest_data.values {
        match data {
            SubstValue::ByteArray(bytes) => {
                writeln!(
                    source_unittest,
                    "static uint8_t g_{var_name}[] = {{ {} }};",
                    bytes
                        .iter()
                        .map(|x| format!("{:#02x}", x))
                        .collect::<Vec<_>>()
                        .join(", ")
                )?;
            }
            SubstValue::String(s) => {
                let s = s.chars().map(|c| format!("'{c}'")).join(", ");
                writeln!(source_unittest, "static char g_{var_name}[] = {{{s}}};")?
            }

            SubstValue::Uint32(val) => writeln!(source_unittest, "uint32_t g_{var_name} = {val};")?,
            SubstValue::Boolean(val) => writeln!(source_unittest, "bool g_{var_name} = {val};")?,
        }
    }
    // Generate structure to hold the TBS data.
    source_unittest.push('\n');
    writeln!(source_unittest, "{tbs_value_struct_name} g_tbs_values = {{")?;
    source_unittest.push_str(&generate_value_struct_assignment(tbs_vars)?);
    source_unittest.push_str("};\n");
    // Generate buffer for the TBS data.
    source_unittest.push('\n');
    writeln!(source_unittest, "uint8_t g_tbs[{max_tbs_size}];")?;
    // Generate structure to hold the certificate data.
    source_unittest.push('\n');
    writeln!(source_unittest, "{sig_value_struct_name} g_sig_values = {{")?;
    source_unittest.push_str(&generate_value_struct_assignment(sig_vars)?);
    source_unittest.push_str("};\n");
    // Generate buffer for the certificate data.
    source_unittest.push('\n');
    writeln!(source_unittest, "uint8_t g_cert_data[{max_cert_size}];\n")?;
    // Generate expected result.
    writeln!(
        source_unittest,
        "const uint8_t kExpectedCert[{}] = {{ {} }};\n",
        expected_cert.len(),
        expected_cert
            .iter()
            .map(|x| format!("0x{:02x}", x))
            .collect::<Vec<_>>()
            .join(", ")
    )?;
    source_unittest.push('\n');

    // Comment out tbs size setter if the size is constant.
    let no_tbs_size = if min_tbs_size == max_tbs_size {
        "// "
    } else {
        ""
    };

    // Generate the body of the test.
    source_unittest.push_str(&format! { r#"
            size_t tbs_size = sizeof(g_tbs);
            EXPECT_EQ(kErrorOk, {generate_tbs_fn_name}(&g_tbs_values, g_tbs, &tbs_size));
            EXPECT_GE(tbs_size, {min_tbs_size});
            EXPECT_LE(tbs_size, {max_tbs_size});

            {no_tbs_size} g_sig_values.tbs_size = tbs_size;

            size_t cert_size = sizeof(g_cert_data);
            EXPECT_EQ(kErrorOk, {generate_cert_fn_name}(&g_sig_values, g_cert_data, &cert_size));
            EXPECT_EQ(cert_size, sizeof(kExpectedCert));
            printf("Generated cert: \n");
            for (size_t i=0; i<cert_size; i++) {{
              printf("%02x, ", g_cert_data[i]);
            }}
            printf("\n");
            EXPECT_EQ(0, memcmp(g_cert_data, kExpectedCert, cert_size));
        "#,
    });

    source_unittest.push_str(
        "
        }
    ",
    );

    Ok(source_unittest)
}

// Generate a structure holding the value of the variables.
fn generate_value_struct(
    value_struct_name: &str,
    variables: &IndexMap<String, VariableType>,
) -> String {
    let mut source = String::new();
    writeln!(source, "typedef struct {value_struct_name} {{").unwrap();
    for (var_name, var_type) in variables {
        let (_, struct_def) = c_variable_info(var_name, "", var_type);
        source.push_str(&struct_def);
    }
    writeln!(source, "}} {value_struct_name}_t;\n").unwrap();
    source
}

// Generate an assignment of a structure holding the values of the variables.
// This is used in the unittest to fill the TBS and sig structures.
fn generate_value_struct_assignment(variables: &IndexMap<String, VariableType>) -> Result<String> {
    let mut source = String::new();
    for (var_name, var_type) in variables {
        let (codegen, _) = c_variable_info(var_name, "", var_type);
        // The TBS variable is special
        match codegen {
            VariableCodegenInfo::Pointer {
                ptr_expr,
                size_expr,
            } => {
                writeln!(source, ".{ptr_expr} = g_{var_name},")?;
                if !var_type.has_constant_array_size() {
                    writeln!(source, ".{size_expr} = sizeof(g_{var_name}),")?;
                }
            }
            VariableCodegenInfo::Uint32 { value_expr }
            | VariableCodegenInfo::Boolean { value_expr } => {
                writeln!(source, ".{value_expr} = g_{var_name},\n")?;
            }
        }
    }
    Ok(source)
}

// Decide if a variable appears in a signature field (if not, it is in the TBS).
fn var_appears_in_sig(var_name: &str, sig: &Signature) -> bool {
    match sig {
        Signature::EcdsaWithSha256 { value } => {
            let Some(EcdsaSignature { r, s }) = value else {
                return false;
            };
            r.refers_to(var_name) || s.refers_to(var_name)
        }
    }
}

#[derive(Debug, PartialEq)]
enum CertificateComponent {
    Certificate,
    Tbs,
}

/// Generate a function that generates a TBS/cert.
///
/// This functions returns the header definition, the generated implementation
/// code and the produced TBS/cert size range.
fn generate_builder(
    component: CertificateComponent,
    fn_name: &str,
    fn_params_str: &str,
    variables: &IndexMap<String, VariableType>,
    build: impl FnOnce(&mut codegen::Codegen) -> Result<()>,
) -> Result<(String, CodegenOutput)> {
    let get_var_info = |var_name: &str| -> Result<VariableInfo> {
        let var_type = variables
            .get(var_name)
            .with_context(|| format!("could not find variable '{var_name}'"))
            .copied()?;
        let (codegen, _) = c_variable_info(var_name, "values->", &var_type);
        Ok(VariableInfo { var_type, codegen })
    };
    let generate_fn_def: String;
    let mut generated_code: CodegenOutput;
    if component == CertificateComponent::Tbs {
        generate_fn_def = indoc::formatdoc! { r#"
        /**
         * Generates a TBS certificate.
         *
         * @param values Pointer to a structure giving the values to use to generate the TBS
         * portion of the certificate.
         * @param[out] out_buf Pointer to a user-allocated buffer that will contain the TBS portion of
         * the certificate.
         * @param[in,out] inout_size Pointer to an integer holding the size of
         * the provided buffer; this value will be updated to reflect the actual size of
         * the output.
         * @return The result of the operation.
         */
        rom_error_t {fn_name}({fn_params_str});

        "#
        };
        generated_code = codegen::Codegen::generate(
            /* buf_name */ "out_buf",
            /* buf_size_name */ "inout_size",
            &get_var_info,
            build,
        )?;
    } else {
        generate_fn_def = indoc::formatdoc! { r#"
        /**
         * Generates an endorsed certificate from a TBS certificate and a signature.
         *
         * @param values Pointer to a structure giving the values to use to generate the
         * certificate (TBS and signature).
         * @param[out] out_buf Pointer to a user-allocated buffer that will contain the
         * result.
         * @param[in,out] inout_size Pointer to an integer holding the size of
         * the provided buffer, this value will be updated to reflect the actual size of
         * the output.
         * @return The result of the operation.
         */
        rom_error_t {fn_name}({fn_params_str});

        "#
        };
        generated_code = codegen::Codegen::generate(
            /* buf_name */ "out_buf",
            /* buf_size_name */ "inout_size",
            &get_var_info,
            build,
        )?;
    }

    let mut generate_fn_impl = String::new();
    writeln!(
        generate_fn_impl,
        "rom_error_t {fn_name}({fn_params_str}) {{"
    )?;
    generate_fn_impl.push_str(&generated_code.code);
    generate_fn_impl.push_str("  return kErrorOk;\n");
    generate_fn_impl.push_str("}\n\n");
    generated_code.code = generate_fn_impl;

    Ok((generate_fn_def, generated_code))
}

// Decide whether a integer should use a special C type instead
// of being represented by a big-endian byte array.
fn c_integer_for_length(size: usize) -> Option<&'static str> {
    match size {
        4 => Some("uint32_t"),
        _ => None,
    }
}

// Return information about a variable (codegen info, definition in struct).
fn c_variable_info(
    name: &str,
    struct_expr: &str,
    var_type: &VariableType,
) -> (VariableCodegenInfo, String) {
    match var_type {
        VariableType::ByteArray { .. } => {
            if var_type.has_constant_array_size() {
                (
                    VariableCodegenInfo::Pointer {
                        ptr_expr: format!("{struct_expr}{name}"),
                        size_expr: format!("{}", var_type.size()),
                    },
                    indoc::formatdoc! {r#"
                        // Pointer to an array of bytes.
                        uint8_t *{name};
                    "#
                    },
                )
            } else {
                (
                    VariableCodegenInfo::Pointer {
                        ptr_expr: format!("{struct_expr}{name}"),
                        size_expr: format!("{struct_expr}{name}_size"),
                    },
                    indoc::formatdoc! {r#"
                        // Pointer to an array of bytes.
                        uint8_t *{name};
                        // Size of this array in bytes.
                        size_t {name}_size;
                    "#
                    },
                )
            }
        }
        VariableType::Integer { .. } => match c_integer_for_length(var_type.size()) {
            Some(c_type) => (
                VariableCodegenInfo::Uint32 {
                    value_expr: format!("{struct_expr}{name}"),
                },
                format!("    {c_type} {name};\n"),
            ),
            None => {
                if var_type.has_constant_array_size() {
                    (
                        VariableCodegenInfo::Pointer {
                            ptr_expr: format!("{struct_expr}{name}"),
                            size_expr: format!("{}", var_type.size()),
                        },
                        indoc::formatdoc! {r#"
                            // Pointer to an unsigned big-endian in integer.
                            uint8_t *{name};
                        "#
                        },
                    )
                } else {
                    (
                        VariableCodegenInfo::Pointer {
                            ptr_expr: format!("{struct_expr}{name}"),
                            size_expr: format!("{struct_expr}{name}_size"),
                        },
                        indoc::formatdoc! {r#"
                            // Pointer to an unsigned big-endian in integer.
                            uint8_t *{name};
                            // Size of this array in bytes.
                            size_t {name}_size;
                        "#
                        },
                    )
                }
            }
        },
        VariableType::String { .. } => {
            if var_type.has_constant_array_size() {
                (
                    VariableCodegenInfo::Pointer {
                        ptr_expr: format!("{struct_expr}{name}"),
                        size_expr: format!("{}", var_type.size()),
                    },
                    indoc::formatdoc! {r#"
                        // Pointer to a (not necessarily zero-terminated) string.
                        char *{name};
                    "#
                    },
                )
            } else {
                (
                    VariableCodegenInfo::Pointer {
                        ptr_expr: format!("{struct_expr}{name}"),
                        size_expr: format!("{struct_expr}{name}_len"),
                    },
                    indoc::formatdoc! {r#"
                        // Pointer to a (not necessarily zero-terminated) string.
                        char *{name};
                        // Length of this string.
                        size_t {name}_len;
                    "#
                    },
                )
            }
        }
        VariableType::Boolean => (
            VariableCodegenInfo::Boolean {
                value_expr: format!("{struct_expr}{name}"),
            },
            format!("bool {name};\n"),
        ),
    }
}
