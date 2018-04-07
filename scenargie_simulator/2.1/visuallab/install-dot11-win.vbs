' install-dot11-win.vbs
' Scenargie Dot Eleven Module Installer for Windows

Set shell = WScript.CreateObject("WScript.Shell")
Root = shell.RegRead("HKCU\SOFTWARE\SpaceTimeEng\Scenargie.exe\Root")
RootData = Root + "\data\"
RootSample = Root + "\sample\"

Set fs = CreateObject("Scripting.FileSystemObject")

fs.CopyFolder "data\dot11", RootData

fs.CopyFile "sample\dot11modes.ber", RootSample

If fs.FileExists("sample\dot11admodes.ber") Then
    fs.CopyFile "sample\dot11admodes.ber", RootSample
End If

If fs.FileExists("sample\dot11admodes_chmodel1.ber") Then
    fs.CopyFile "sample\dot11admodes_chmodel1.ber", RootSample
End If

If fs.FileExists("sample\dot11admodes_chmodel2.ber") Then
    fs.CopyFile "sample\dot11admodes_chmodel2.ber", RootSample
End If

If fs.FileExists("sample\dot11admodes_chmodel3.ber") Then
    fs.CopyFile "sample\dot11admodes_chmodel3.ber", RootSample
End If

If fs.FileExists("sample\dot11admodes_chmodel4.ber") Then
    fs.CopyFile "sample\dot11admodes_chmodel4.ber", RootSample
End If

If fs.FileExists("sample\dot11admodes_chmodel5.ber") Then
    fs.CopyFile "sample\dot11admodes_chmodel5.ber", RootSample
End If

If fs.FileExists("sample\dot11admodes_chmodel6.ber") Then
    fs.CopyFile "sample\dot11admodes_chmodel6.ber", RootSample
End If

If fs.FileExists("sample\dot11ahmodes.ber") Then
    fs.CopyFile "sample\dot11ahmodes.ber", RootSample
End If

