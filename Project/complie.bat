@echo off
REM �����UV���ó�Keil��װ��ַ
set UV=F:\Keil_v5\UV4\UV4.exe
if "%1" == "" goto NOP
set UV_PRO_PATH=%cd%\USER\%1.uvprojx
if "%2" == "-b" (
    echo Init building ... 
    %UV% -j0 -b %UV_PRO_PATH% -o %cd%\build_log.txt 
    type build_log.txt
) else if "%2" == "-f" (
    echo Init flashing ...  
    %UV% -j0 -f %UV_PRO_PATH% -o %cd%\build_log.txt 
    type build_log.txt
) else (
    echo Init building ... 
    %UV% -j0 -b %UV_PRO_PATH% -o %cd%\build_log.txt 
    type build_log.txt
    echo Init flashing ... 
    %UV% -j0 -f %UV_PRO_PATH% -o %cd%\build_log.txt 
    type build_log.txt
)
if "%3" == "-f" (
    echo Init flashing ... 
    %UV% -j0 -f %UV_PRO_PATH% -o %cd%\build_log.txt 
    type build_log.txt
)
goto end

:NOP
echo No ProjectName
goto end

:end
exit
