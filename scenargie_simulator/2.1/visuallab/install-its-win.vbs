' install-its-win.vbs
' Scenargie ITS Extension Module Installer for Windows

Set shell = WScript.CreateObject("WScript.Shell")
Root = shell.RegRead("HKCU\SOFTWARE\SpaceTimeEng\Scenargie.exe\Root")
RootData = Root + "\data\"

Set fs = CreateObject("Scripting.FileSystemObject")

fs.CopyFolder "data\its", RootData
