//! C-compatible entry points for complete archive compression and restoration.
//!
//! Both operations share a boundary that validates pointers and size limits,
//! contains Rust panics, stores per-thread diagnostics, and transfers output
//! buffers to the caller with a matching library-owned free function.
use std::{
    cell::RefCell,
    ffi::{c_char, CString},
    panic::{catch_unwind, AssertUnwindSafe},
    ptr, slice,
};

const OK: i32 = 0;
const INVALID_ARGUMENT: i32 = 1;
const CODEC_ERROR: i32 = 2;
const PANIC: i32 = 3;
const MAX_INPUT_SIZE: usize = 1 << 30;

thread_local! {
    static LAST_ERROR: RefCell<CString> = RefCell::new(CString::default());
}

fn set_last_error(message: &str) {
    let clean = message.replace('\0', "\\0");
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = CString::new(clean).unwrap_or_default();
    });
}

unsafe fn transform(
    input: *const u8,
    input_len: usize,
    output: *mut *mut u8,
    output_len: *mut usize,
    operation: fn(&[u8]) -> Result<Vec<u8>, String>,
) -> i32 {
    LAST_ERROR.with(|slot| *slot.borrow_mut() = CString::default());
    if !output.is_null() {
        *output = ptr::null_mut();
    }
    if !output_len.is_null() {
        *output_len = 0;
    }
    if output.is_null() || output_len.is_null() || (input.is_null() && input_len != 0) {
        set_last_error("null pointer passed to codec");
        return INVALID_ARGUMENT;
    }
    if input_len > MAX_INPUT_SIZE {
        set_last_error("input exceeds the 1 GiB API limit");
        return INVALID_ARGUMENT;
    }

    let bytes = if input_len == 0 {
        &[]
    } else {
        // SAFETY: the caller promises `input` points to `input_len` readable bytes.
        slice::from_raw_parts(input, input_len)
    };
    match catch_unwind(AssertUnwindSafe(|| operation(bytes))) {
        Ok(Ok(encoded)) => {
            let boxed = encoded.into_boxed_slice();
            *output_len = boxed.len();
            // SAFETY: the paired free function reconstructs the boxed slice.
            *output = Box::into_raw(boxed) as *mut u8;
            OK
        }
        Ok(Err(error)) => {
            set_last_error(&error);
            CODEC_ERROR
        }
        Err(_) => {
            set_last_error("codec panicked; panic was contained at the C ABI boundary");
            PANIC
        }
    }
}

/// ABI version for callers that need to check compatibility.
#[no_mangle]
pub extern "C" fn arw_archive_abi_version() -> u32 {
    1
}

/// Compress one supported Sony ARW2 byte buffer into an AAA1 archive.
///
/// Returns 0 on success. `output` must be released with `arw_archive_buffer_free`.
/// On failure, use `arw_archive_last_error` for a thread-local diagnostic string.
///
/// # Safety
/// `input` must reference `input_len` readable bytes unless `input_len` is zero.
/// `output` and `output_len` must be valid writable pointers.
#[no_mangle]
pub unsafe extern "C" fn arw_archive_compress(
    input: *const u8,
    input_len: usize,
    output: *mut *mut u8,
    output_len: *mut usize,
) -> i32 {
    transform(
        input,
        input_len,
        output,
        output_len,
        crate::archive::pack_archive,
    )
}

/// Restore one AAA1 archive to the original ARW2 bytes and validate its digest.
///
/// # Safety
/// Pointer requirements and output ownership are the same as `arw_archive_compress`.
#[no_mangle]
pub unsafe extern "C" fn arw_archive_decompress(
    input: *const u8,
    input_len: usize,
    output: *mut *mut u8,
    output_len: *mut usize,
) -> i32 {
    transform(
        input,
        input_len,
        output,
        output_len,
        crate::archive::unpack_archive,
    )
}

/// Free a buffer returned by either compression or decompression.
///
/// # Safety
/// `buffer` and `length` must be the exact pair returned by this library and
/// must not have been freed already.
#[no_mangle]
pub unsafe extern "C" fn arw_archive_buffer_free(buffer: *mut u8, length: usize) {
    if !buffer.is_null() {
        let slice = ptr::slice_from_raw_parts_mut(buffer, length);
        // SAFETY: this pointer-length pair was allocated by `transform`.
        drop(Box::from_raw(slice));
    }
}

/// Return the most recent diagnostic for this thread; valid until the next ABI call.
#[no_mangle]
pub extern "C" fn arw_archive_last_error() -> *const c_char {
    LAST_ERROR.with(|slot| slot.borrow().as_ptr())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reports_invalid_ffi_arguments_without_panicking() {
        let mut output = ptr::null_mut();
        let mut output_len = usize::MAX;
        let status = unsafe { arw_archive_compress(ptr::null(), 1, &mut output, &mut output_len) };
        assert_eq!(status, INVALID_ARGUMENT);
        assert!(output.is_null());
        assert_eq!(output_len, 0);
        assert!(!arw_archive_last_error().is_null());
    }
}
