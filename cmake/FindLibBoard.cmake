# - Try to find LibBoard
# Once done this will define
#  LibBoard_FOUND - System has LibXml2
#  LibBoard_INCLUDE_DIRS - The LibXml2 include directories
#  LibBoard_LIBRARIES - The libraries needed to use LibXml2
#  LibBoard_DEFINITIONS - Compiler switches required for using LibXml2

find_path(LibBoard_INCLUDE_DIR
          Board.h
          HINTS /usr/local/include
)

find_library(LibBoard_LIBRARY
             NAMES board
             HINTS /usr/local/lib
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibBoard
                                  DEFAULT_MSG
                                  LibBoard_LIBRARY
                                  LibBoard_INCLUDE_DIR)

mark_as_advanced(LibBoard_INCLUDE_DIR LibBoard_LIBRARY)

set(LibBoard_LIBRARIES ${LibBoard_LIBRARY} )
set(LibBoard_INCLUDE_DIRS ${LibBoard_INCLUDE_DIR} )
