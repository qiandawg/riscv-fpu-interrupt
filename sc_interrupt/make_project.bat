@echo off
for /d %%V in ("C:\Xilinx\Vivado\20*") do set LATEST=%%V
echo Using %LATEST%
call "%LATEST%\settings64.bat"
vivado -nolog -nojournal -mode batch -source ./tcl/make_project.tcl
call vivado -nolog -nojournal -mode=batch -source=./tcl/impl.tcl