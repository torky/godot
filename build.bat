@echo off

if not defined NUGET_LOCAL_SOURCE (
    set NUGET_LOCAL_SOURCE=%USERPROFILE%/MyLocalNugetSource
)

scons platform=windows target=editor module_mono_enabled=yes
if %errorlevel% neq 0 exit /b %errorlevel%

cd bin
godot.windows.editor.x86_64.mono.exe --headless --generate-mono-glue ../modules/mono/glue
if %errorlevel% neq 0 ( cd .. & exit /b %errorlevel% )

cd ..
python ./modules/mono/build_scripts/build_assemblies.py --godot-output-dir=./bin --godot-platform=windows --push-nupkgs-local %NUGET_LOCAL_SOURCE%
