@echo off
setlocal enabledelayedexpansion

:: Arduino ���� ���� �⺻ ���
set "arduinoBuildPath=C:\Users\tyback\AppData\Local\arduino\sketches"
set "projectPath=C:\Users\tyback\OneDrive\����\Arduino\bin"

:: ���� �ֽ� ������ ã�� ���� ���� �ʱ�ȭ
set "latestFolder="
set "latestTime=0"

:: ���� ����� ��ȸ�ϸ� �ֽ� ��¥�� ���� ã��
for /d %%F in ("%arduinoBuildPath%\*") do (
    for %%G in ("%%F") do (
        set "folderTime=%%~tG"
        set "folderName=%%F"

        :: ������ ������ ���� �ð��� ���� �ֽ����� Ȯ��
        if !folderTime! GTR !latestTime! (
            set "latestTime=!folderTime!"
            set "latestFolder=!folderName!"
        )
    )
)

:: �ֽ� �������� ���̳ʸ� ���� ����
if not "!latestFolder!"=="" (
    echo ?? ���� �ֱ� ������ ����: !latestFolder!
    copy /Y "!latestFolder!\*.bin" "%projectPath%\"
    echo ? �ֽ� ���̳ʸ� ������ ������Ʈ ������ �̵� �Ϸ�!
) else (
    echo ? �ֽ� ������ ã�� �� �����ϴ�. �������� ���� �����ϼ���.
)

pause
