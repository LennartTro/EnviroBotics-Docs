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
For those of you who have their windows encrypted with bitlocker you might have to enter your bitlocker key the next time you boot windows after you have installed Ubuntu. You should save your bitlocker key (you can find it here). Alternatively you can turn off bitlocker encryption.
:::

### Step 1 – Download Ubuntu & Step 2 – Create a Bootable USB Stick (from Windows)

Is already done and will be provided

---

### Step 3 – Prepare Windows for Dual Boot

Before installing Ubuntu, free up some space on your hard drive:

1. Press Win + X → choose Disk Management.

2. Right-click on your main Windows partition (e.g., C:) → choose Shrink Volume.

3. Shrink by at least 20,000 MB (20 GB) or more for Ubuntu.

4. Leave the new partition as Unallocated.
---

### Step 4 – Boot from USB
1. Reboot your PC.

2. While it starts, press the boot menu key (usually F12, Esc, F2, Del, or Enter – varies by manufacturer).

3. Now you want to boot from the UBUNTU USB stick.

4. In the Grub menu, choose Ubuntu (probably the first option). This will boot Ubuntu from the USB stick after a quick file system check.
---

### Step 5 – Install Ubuntu

1. Once booted, you will get the option to either Try Ubuntu or Install Ubuntu immediately. If you decide to try Ubuntu before installing, you will find an icon for installing Ubuntu on the desktop. Double click it to launch the installation wizard. If you don’t want to try Ubuntu first, you can click on Install Ubuntu to start the installation wizard.

2. We highly recommend setting the language to English.

3. Choose the option to install additional drivers. It is also a good idea to connect to a nearby WiFi or Ethernet.
4. In the last step of the installation, the wizard asks you if you want to install Ubuntu alongside windows because it detects your windows installation and the free disk space we created before. Choose this option and click Install Now.
:::warning
Do **NOT** choose the **Erase disk and install Ubuntu** option! This will delete your Windows installation!
:::
5. Reboot.

You should now have a running Ubuntu version on your laptop!

This is the time to check that your Windows partition still runs, too. As mentioned at the beginning, you might be asked for the bitlocker key when booting Windows again for the first time.