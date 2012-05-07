include(FindPkgMacros)
include(PreprocessorUtils)

findpkg_begin(OgreProcedural)

getenv_path(OgreProcedural_HOME)
getenv_path(PROGRAMFILES)

# construct search paths from environmental hints and
# OS specific guesses
if (WIN32)
  set(OgreProcedural_PREFIX_GUESSES
    ${ENV_PROGRAMFILES}/OgreProcedural
    C:/OgreProceduralSDK
  )
elseif (UNIX)
  set(OgreProcedural_PREFIX_GUESSES
    /opt/OgreProcedural
    /opt/OgreProcedural
    /usr/lib${LIB_SUFFIX}/OgreProcedural
    /usr/lib${LIB_SUFFIX}/OgreProcedural
    /usr/local/lib${LIB_SUFFIX}/OgreProcedural
    /usr/local/lib${LIB_SUFFIX}/OgreProcedural
    /usr/local/
    /usr/local/
    $ENV{HOME}/OgreProcedural
    $ENV{HOME}/OgreProcedural
  )
endif ()

set(OgreProcedural_PREFIX_PATH
 ${OgreProcedural_HOME} ${OgreProcedural_SDK} ${ENV_OgreProcedural_HOME} ${ENV_OgreProcedural_SDK}
  ${OgreProcedural_PREFIX_GUESSES}
)
create_search_paths(OgreProcedural)

set(OgreProcedural_LIBRARY_NAMES "OgreProcedural")
get_debug_names(OgreProcedural_LIBRARY_NAMES)

find_path(OgreProcedural_INCLUDE_DIR NAMES Procedural.h HINTS ${OgreProcedural_CONFIG_INCLUDE_DIR} ${OgreProcedural_INC_SEARCH_PATH} ${OgreProcedural_FRAMEWORK_INCLUDES} ${OgreProcedural_PKGC_INCLUDE_DIRS} PATH_SUFFIXES "PROCEDURAL")

find_library(OgreProcedural_LIBRARY_REL NAMES ${OgreProcedural_LIBRARY_NAMES} HINTS ${OgreProcedural_LIB_SEARCH_PATH} PATH_SUFFIXES "" "release" "relwithdebinfo" "minsizerel")
find_library(OgreProcedural_LIBRARY_DBG NAMES ${OgreProcedural_LIBRARY_NAMES_DBG} HINTS ${OgreProcedural_LIB_SEARCH_PATH} PATH_SUFFIXES "" "debug")

make_library_set(OgreProcedural_LIBRARY)

findpkg_finish(OgreProcedural)
