---
id: IDE Setup
title: IDE Setup
---

# IDE Setup

## Installing Visual Studio Code on Ubuntu

Here are two recommended installation methods to get VS Code running on Ubuntu.

### Installing VS Code via snap (easier):
Please check your snap version: 
```bash
snap version
```
If there is no version - install snap:

```bash
sudo apt update
sudo apt install snapd
```

Now install VS Code:
```bash
sudo snap install --classic code
```

### or Install via Official `.deb` Package

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
---
# 🖥️ Install Terminator on Ubuntu

**[Terminator](https://gnome-terminator.org/)** is a powerful terminal emulator that allows you to split windows and manage multiple terminal sessions efficiently — perfect for robotics workflows where you often need multiple terminals open.

---

## ✅ Installation

Open a terminal and run:

```bash
sudo apt update
sudo apt install terminator
```

Terminator should now be installed on your system.
### How to Start Terminator

To launch Terminator:

```bash
terminator

```
:::tip Basic Usage Tips – Terminator Shortcuts

Here are some useful keyboard shortcuts for working with [**Terminator**](https://gnome-terminator.org/):

| Action               | Shortcut               |
|----------------------|------------------------|
| Split Horizontally   | `Ctrl + Shift + O`     |
| Split Vertically     | `Ctrl + Shift + E`     |
| Move Between Panes   | `Ctrl + Shift + ↑↓←→`  |
| Open New Tab         | `Ctrl + Shift + T`     |
| Close Pane           | `Ctrl + Shift + W`     |

:::


### Set Terminator as Default Terminal
If you'd like Terminator to open whenever a terminal is requested (e.g. via right-click in file explorer), you can set it as the default terminal:
```bash
sudo update-alternatives --config x-terminal-emulator

```
You’ll be prompted with a list — choose the number corresponding to Terminator.
## Test Terminator Setup
Open a new pane: `Ctrl + Shift + T`