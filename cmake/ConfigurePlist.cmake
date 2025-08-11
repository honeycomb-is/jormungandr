file(READ "${INPUT}" _plist_in)
set(_plist_out "${_plist_in}")
string(REPLACE "@BUNDLE_NAME@" "${BUNDLE_NAME}" _plist_out "${_plist_out}")
string(REPLACE "@BUNDLE_ID@" "${BUNDLE_ID}" _plist_out "${_plist_out}")
string(REPLACE "@BUNDLE_VERSION@" "${BUNDLE_VERSION}" _plist_out "${_plist_out}")
string(REPLACE "@MIN_SYS_VER@" "${MIN_SYS_VER}" _plist_out "${_plist_out}")
file(WRITE "${OUTPUT}" "${_plist_out}")

