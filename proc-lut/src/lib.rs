use proc_macro::TokenStream;
use quote::quote;

#[proc_macro]
pub fn lut_100(_input: TokenStream) -> TokenStream {
    let mut table = [0; 101];
    let mut i = 0;
    while i <= 100 {
        table[i] = 10.0f64.powf(i as f64 / 20.0).round() as u32;
        i += 1;
    }

    quote! { [#(#table),*] }.into()
}
