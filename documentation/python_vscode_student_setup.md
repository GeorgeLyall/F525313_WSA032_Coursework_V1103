# Python + VS Code: Python Setup Guide

This guide describes the **supported and expected setup** for the coursework.
Please follow the steps **in order**. If you do, everything should work.

As discussed in the week 7 notes, you should run a module in the robot ecosystem using

```
python.exe -m robots.ecosystem_operation
```
`-m` is a command line option instructing Python to treat `robots.ecosystem_operation` as an **import path**, not a file path. This loads executes code through Python's import system, respecting all the dependences between the modules and libraries.

This is the correct and expected way to run this project. You `robot_optimisation` module can be run in the same way.
```
python.exe -m robots.robot_optimisation
```

In contrast, running a module as a script by clicking the `Run Python File` button on the top right means the file will not properly know the location of any dependent files.

>
>  Note that method might work for a testing a bit of code in a file with no dependences.
>

_Follow this guide to make sure Python is set up. Some steps may require you to have **admin** rights on the device._

---

## 1. Check whether the *terminal* can run Python

VS Code can sometimes run Python even if the terminal is not set up yet.
For this module, **the terminal must work**.

### How to open the terminal

- Open **VS Code**
- From the top menu, choose **Terminal → New Terminal**

In the terminal window that appears, type:

```
python --version
```

If you see something like:

```
Python 3.12.x
```

✅ The terminal is correctly set up

If instead you see a message saying Python is not installed:

❌ This means **Windows cannot find Python yet**, even if VS Code can run files.
Continue to Section 2.

---
## 2. Installing Python

If required install Python. The best route is using **the standard Python install from python.org**.

We struggle to support alternative environment managers such as uv, Conda, or Condor with which we may not be familiar. You can revert to a standard install by uninstalling these packages, this will not damage your coursework folder!

### Install steps

1. Go to:
   https://www.python.org/downloads/
2. Download **Python 3.12**  (New version are available - this is most stable)
3. Run the installer and **tick this box**:
   - ✅ *Add Python to PATH*
4. Click **Install**

> **Note**  
> Python may install into a user-specific folder (for example inside `AppData`).  
> This is completely fine.

After installation:
- **Close and reopen VS Code**
- Open the terminal again
- Re-run:

```
python --version
```

Once this works, continue.

---

## 3. Install the VS Code Python extension

This module uses the **VS Code Python extension by Microsoft**.
This is our supported toolchain.

### Install steps

1. In VS Code, open **Extensions** (left sidebar)
2. Search for **Python**
3. Install **“Python” by Microsoft**

This extension provides:
- running Python files
- virtual environments
- debugging support
- interpreter selection

---

## 4. Open your coursework folder

You should already have a coursework folder provided for this module.

### Important checks

- Ensure the folder is in a sensible location, for example:
  - your **OneDrive Documents** folder
  - specifically for module **25WSA032**
- If the folder was downloaded as a `.zip` file:
  - **extract (unzip) it fully**
  - open the extracted folder, not the zip

### Open the folder

- In VS Code, choose **File → Open Folder**
- Select your coursework folder

---

## 5. Create a virtual environment (venv)

Once `python --version` works in the terminal, create a virtual environment. This can greatly simplify your python experience. 

In the terminal:

```
python -m venv .venv
```

This creates a folder called `.venv` inside your coursework folder.

---

## 6. Activate and select the venv

### Activate the environment

run the powershell command `active.ps1` which will be in the scripts folder :

**Windows (PowerShell):**
```
.venv\Scripts\Activate
```

**macOS / Linux:**
```
source .venv/bin/activate
```
Afterwards you should see `(.venv)` at the start of the terminal prompt. Note that gain you may not have permission to run powershell commands due to a long path name. Run powershell (outside of VSCode) as administrator. Then run this command at the prompt:
```
New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force
```


### Select the interpreter in VS Code

VS Code may prompt you automatically to select the new environment.

If it does:
- **Select the interpreter that contains `.venv`**

If not:
- Press **Ctrl + Shift + P**
- Run **Python: Select Interpreter**
- Choose the interpreter inside `.venv`

---

## 7. Install required packages

Your coursework folder includes a file called:

```
requirements.txt
```

This lists all required packages.

With the venv active, run:

```
python -m pip install -r requirements.txt
```

This installs everything needed for the project.

---

## 8. Run the project demonstration

You can now run the example program.

From the terminal, in the coursework folder:

```
python.exe -m robots.ecosystem_operation
```
---

## If things still do not work

If you see errors mentioning tools such as **uv**, **conda**, or other environment managers:

> You are using a non-standard Python setup that is **not supported** for this module.

### Fix

1. Install Python from https://www.python.org/downloads/
   - Python **3.12** recommended
2. Ensure **Add Python to PATH** is selected
3. Restart VS Code
4. Follow this guide **from the start**, exactly

---

## One rule to remember

> **Get `python --version` working in the terminal first.**  
> **Then create and use `.venv`.**

If you follow that order, everything else will work.
