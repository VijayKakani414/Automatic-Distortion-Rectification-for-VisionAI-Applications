###################################
#   Setup MATLAB variables here
###################################
set(MATLAB_DIR 
    "C:/Program Files/MATLAB/R2018a"
    CACHE PATH "Set Matlab installation path"
)
set(MATLAB_RELEASE 
    R2017b
    CACHE STRING "Set Matlab release string"
)

set(MATLAB_CXX_FLAGS "/Zp8 /GR /W3 /EHs /nologo /MD /Z7")
set(MATLAB_INCLUDE_DIRS 
    ${MATLAB_DIR}/extern/include
    ${MATLAB_DIR}/simulink/include
)
set(MATLAB_LIB_DIRS ${MATLAB_DIR}/extern/lib/win64/microsoft)
set(MATLAB_LIBS 
    libmx.lib 
    libmex.lib 
    libmat.lib 
    kernel32.lib 
    user32.lib 
    gdi32.lib 
    winspool.lib 
    comdlg32.lib 
    advapi32.lib 
    shell32.lib 
    ole32.lib 
    oleaut32.lib
    uuid.lib
    odbc32.lib
    odbccp32.lib
    libMatlabDataArray.lib
    libMatlabEngine.lib
    libeng.lib
    libMatlabDataArray.lib
    libMatlabEngine.lib
)
set(MATLAB_CXX_DEFS
    /DMATLAB_DEFAULT_RELEASE=${MATLAB_RELEASE}
    /DUSE_MEX_CMD
    /D_CRT_SECURE_NO_DEPRECATE
    /D_SCL_SECURE_NO_DEPRECATE
    /D_SECURE_SCL=0
)
