---
id: Install Ubuntu
title: Install Ubuntu
---

# Dual-Boot Setup: Install Ubuntu alongside Windows

This section should guide you through the process of installing Ubuntu alongisde Windows on your machine. If you prefer to install Ubuntu inside a Virtual Machine, you can skip this section and continue with Virtual Machine.
:::important
Since we are going to install a new/second operating system on your device, that might be a good opportunity to create a backup just in case anything goes wrong! We are not responsible for any damage or loss of data that might result from any of the following instructions.
:::

:::note
For those of you who have their windows encrypted with bitlocker you might have to enter your bitlocker key the next time you boot windows after you have installed Ubuntu. You should save your bitlocker key. Alternatively you can turn off bitlocker encryption.
The link for the manual on how to find it can be found here: https://support.microsoft.com/en-au/windows/find-your-bitlocker-recovery-key-6b71ad27-0b89-ea08-f143-056f5ab347d6
:::

### Step 1 – Download Ubuntu & Step 2 – Create a Bootable USB Stick (from Windows)

Is already done and will be provided

---

### Step 3 – Prepare Windows for Dual Boot

Before installing Ubuntu, free up some space on your hard drive:

1. Press Win + X → choose Disk Management.

2. Right-click on your main Windows partition (e.g., C:) → choose Shrink Volume.

3. Shrink by at least 30,000 MB (30 GB) or more for Ubuntu.
<img src="/EnviroBotics-Docs/img/windows_disk_manager.png" alt="Concept Overview" style={{ width: '100%', marginBottom: '1rem' }} />


4. Leave the new partition as Unallocated.T he result could look similiar to this:
<img src="/EnviroBotics-Docs/img/windows_disk_manager_result.png" alt="Concept Overview" style={{ width: '100%', marginBottom: '1rem' }} />

---

### Step 4 Disable Secure Boot

1. Reboot your system and enter your BIOS/UEFI during Bootup. To do so, you have to hit a certain key, depending on your hardware. For Dell commputers you probably need to hit F12 during the DELL splashscreen. For Lenovo, this key is Enter. This opens a dialog where you can choose to enter your BIOS settings.
2. Find the settings to disable Secure Boot, save your changes and exit the BIOS/UEFI.

---

### Step 5 – Boot from USB
1. Reboot and select the medium you want to boot from during the splashscreen (again F12 for Dell or Enter for Lenovo). Now you want to boot from the Ubuntu USB stick.
2. In the Grub menu, choose Ubuntu (probably the first option). This will boot Ubuntu from the USB stick after a quick file system check.

---

### Step 6 – Start The Installation Wizard

1. Once booted, you will get the option to either **Try Ubuntu** or **Install Ubuntu immediately**. If you decide to try Ubuntu before installing, you will find an icon for installing Ubuntu on the desktop. Double click it to launch the installation wizard. If you don’t want to try Ubuntu first, you can click on Install Ubuntu to start the installation wizard.
2. We highly recommend setting the language to English.
3. Choose the option to install additional drivers. It is also a good idea to connect to a nearby WiFi or Ethernet.
<img src="/EnviroBotics-Docs/img/ubuntu_additional_drivers.png" alt="Concept Overview" style={{ width: '100%', marginBottom: '1rem' }} />

4. In the last step of the installation, the wizard asks you if you want to install Ubuntu alongside windows because it detects your windows installation and the free disk space we created before. Choose this option and click Install Now.
:::warning
Do **NOT** choose the **Erase disk and install Ubuntu** option! This will delete your Windows installation!
:::
<img src="/EnviroBotics-Docs/img/ubuntu_alongside.png" alt="Concept Overview" style={{ width: '100%', marginBottom: '1rem' }} />

5. Reboot.

You should now have a running Ubuntu version on your laptop!

This is the time to check that your Windows partition still runs, too. As mentioned at the beginning, you might be asked for the bitlocker key when booting Windows again for the first time.

---

## Some Convenience Features
:::note
The following is optional, just some tips/fixes for your convenience.
:::

Now that your dual-boot is set up, you might notice a few changes.

When booting, you will see the Grub menu. By default, Ubuntu is the first option. If you are mostly using Windows in your daily life, this can become tedious. It’s easy to not react quickly enough and to end up booting Ubuntu by accident. Ending up in the seemingly endless rebooting cycle…

While (hopefully!) you will spend a lot of time using Ubuntu this semester, the following explains how to change this back to Windows as the default.

Open the Grub configuration file (for editing this, you need to open with sudo)

```bash
sudo gedit /etc/default/grub
```

A window should open with contents similar to this:

```ini {6} showLineNumbers
# If you change this file, run 'update-grub' afterwards to update
# /boot/grub/grub.cfg.
# For full documentation of the options in this file, see:
#   info -f grub -n 'Simple configuration'

GRUB_DEFAULT=0
GRUB_TIMEOUT_STYLE=hidden
GRUB_TIMEOUT=10
GRUB_DISTRIBUTOR=`lsb_release -i -s 2> /dev/null || echo Debian`
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
GRUB_CMDLINE_LINUX=""

# Uncomment to enable BadRAM filtering, modify to suit your needs
# This works with Linux (no patch required) and with any kernel that obtains
# the memory map information from GRUB (GNU Mach, kernel of FreeBSD ...)
#GRUB_BADRAM="0x01234567,0xfefefefe,0x89abcdef,0xefefefef"

# Uncomment to disable graphical terminal (grub-pc only)
#GRUB_TERMINAL=console

# The resolution used on graphical terminal
# note that you can use only modes which your graphic card supports via VBE
# you can see them in real GRUB with the command `vbeinfo'
#GRUB_GFXMODE=640x480

# Uncomment if you don't want GRUB to pass "root=UUID=xxx" parameter to Linux
#GRUB_DISABLE_LINUX_UUID=true

# Uncomment to disable generation of recovery mode menu entries
#GRUB_DISABLE_RECOVERY="true"

# Uncomment to get a beep at grub start
#GRUB_INIT_TUNE="480 440 1"
```

Find the line setting the GRUB_DEFAULT, highlighted above. Setting this to 0 means that in the Grub menu, the very first option will be the default.

Set this to the number of the Windows option in your Grub menu (starting to count from 0!) You will have to look this up again, but it is likely that Windows is the third option (i.e. GRUB_DEFAULT = 2).

Change the correct line in your configuration file.

After changing this file, update your configuration:
```bash 
update-grub
```
Finally, you might notice a wrong time displayed when switching between Windows and Ubuntu.

In order to fix this, in Ubuntu, simply run:
```bash
timedatectl set-local-rtc 1
```


