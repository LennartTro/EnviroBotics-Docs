---
id: IDE Setup
title: IDE Setup
---

# IDE Setup

## Installing Visual Studio Code on Ubuntu

Here are two recommended installation methods to get VS Code running on Ubuntu.

### Install via Official `.deb` Package

```bash
# Download the latest .deb package from Microsoft (example link)
wget -O ~/vscode.deb https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-arm64
# Alternatively, download manually via your browser
```

```bash
# Install the package
sudo apt install ~/vscode.deb
# Fix any missing dependencies
sudo apt --fix-broken install
```

```bash
# Launch VS Code
code --version
```

## Installing Visual Studio Code on Windows

Visual Studio Code can be installed easily on Windows using the official installer provided by Microsoft.

### Download the Installer

👉 [Download VS Code for Windows (64-bit)](https://code.visualstudio.com/Download)

You can choose between:
- **User Installer** (recommended for single users)
- **System Installer** (for all users on the machine)

> Tip: Use the **User Installer** unless you specifically need system-wide access.

---

### Installation Steps

```text
1. Run the downloaded `.exe` installer.
2. Accept the license agreement.
3. Select the destination folder (or leave default).
4. Check these recommended options:
   - Add "Open with Code" to the context menu
   - Register Code as the default editor for supported file types
   - Add to PATH (important for using `code` in terminal)
5. Click Install and finish setup.
