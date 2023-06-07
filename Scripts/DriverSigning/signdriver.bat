cd ..rem CD to the RepRapFirmware root directory before running this
"C:\Program Files (x86)\Windows Kits\10\bin\x86\Inf2Cat.exe" /driver:Driver /os:XP_X86,Vista_X86,Vista_X64,7_X86,7_X64,8_X86,8_X64,6_3_X86,6_3_X64,10_X86,10_X64
set PASSWORD=
set /P PASSWORD=Private key password: %=%
"C:\Program Files (x86)\Windows Kits\10\bin\x86\signtool.exe" sign /f g:\EscherTechnologies.pfx /p %PASSWORD% /tr http://timestamp.comodoca.com /v Driver/duetinf.cat
"C:\Program Files (x86)\Windows Kits\10\bin\x86\signtool.exe" verify /pa /tw Driver/duetinf.cat