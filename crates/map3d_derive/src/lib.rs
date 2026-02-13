extern crate proc_macro;
extern crate quote;

#[cfg(not(feature = "pyo3"))]
use proc_macro::TokenStream;

/// Needed to mock pyo3 macro attributes in case we're not using the pyo3 feature
#[cfg(not(feature = "pyo3"))]
#[proc_macro_derive(DummyPyO3, attributes(pyo3))]
pub fn derive_dummy_pyo3(_input: TokenStream) -> TokenStream {
    dummy_pyo3::erase_input()
}
#[cfg(not(feature = "pyo3"))]
#[proc_macro_attribute]
pub fn new(_attr: TokenStream, _item: TokenStream) -> TokenStream {
    dummy_pyo3::erase_input()
}
#[cfg(not(feature = "pyo3"))]
#[proc_macro_attribute]
pub fn getter(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let ast = syn::parse_macro_input!(item as syn::Item);
    dummy_pyo3::strip_attributes(&ast)
}
#[cfg(not(feature = "pyo3"))]
#[proc_macro_attribute]
pub fn setter(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let ast = syn::parse_macro_input!(item as syn::Item);
    dummy_pyo3::strip_attributes(&ast)
}
#[cfg(not(feature = "pyo3"))]
#[proc_macro_attribute]
pub fn staticmethod(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let ast = syn::parse_macro_input!(item as syn::Item);
    dummy_pyo3::strip_attributes(&ast)
}
#[cfg(not(feature = "pyo3"))]
#[proc_macro_attribute]
pub fn classattr(_attr: TokenStream, _item: TokenStream) -> TokenStream {
    dummy_pyo3::erase_input()
}
#[cfg(not(feature = "pyo3"))]
#[proc_macro_attribute]
pub fn pyo3(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let ast = syn::parse_macro_input!(item as syn::Item);
    dummy_pyo3::strip_attributes(&ast)
}

#[cfg(not(feature = "pyo3"))]
mod dummy_pyo3 {
    use super::*;

    pub fn strip_attributes(ast: &syn::Item) -> TokenStream {
        quote::quote! {
            #ast
        }
        .into()
    }
    pub fn erase_input() -> TokenStream {
        quote::quote! {}.into()
    }
}
