---
layout: default
title: "Linux"
parent: Tutorials
sort: 3
---

## 1. Bash

**Bash** (Bourne Again SHell) is the command-line interpreter you see in the terminal. It is the language you use to tell the OS what to do.

#### Core Commands:
- `pwd`: **Print Working Directory**. Shows exactly where you are in the folders.
- `ls`: **List**. Shows files in the current folder. (Use `ls -la` to see hidden files).
- `cd [path]`: **Change Directory**. Move into a folder. `cd ..` moves up one level.
- `mkdir [name]`: **Make Directory**. Creates a new folder.
- `sudo`: **SuperUser Do**. Runs a command with "Admin" privileges.
- `cat [file]`: **Concatenate**. Displays the text inside a file in the terminal.

## 2. SSH (Secure Shell)
SSH is used to log into a remote computer (like your TurtleBot 4) securely.

- **The Command:** `ssh username@ip_address`
- **Why use it?** You can't plug a monitor into a robot while it's driving. SSH lets you control it over WiFi.
- **Pro Tip:** If you see a "Host key verification failed" error, it's usually because the robot was re-imaged. Fix it with: `ssh-keygen -R [IP_ADDRESS]`.


## 3. tmux (Terminal Multiplexer)
`tmux` is essential for ROS development and SSH. It allows you to run multiple terminal windows inside one connection and keeps them running even if your WiFi drops. It also allows you to create mutliple terminal windows easily without having to SSH a new terminal every time.

### 3.1. Why it's critical for SSH/ROS:
If you are driving a robot via SSH and your WiFi blips, a normal session will die—and the robot might keep driving! If you use `tmux` (a terminal program), the process keeps running on the robot even if you lose connection.

### 3.2. Basic Keyboard Shortcuts:
Once you start tmux, press your **Prefix** key (`Ctrl+b`) then the key:

- `|` : Split the screen **Horizontally**.
- `-` : Split the screen **Vertically**.
- `c` : Create a **New Tab** (Window).
- `x` : Close the current **Tab**.
- `s` : **Switch Sessions**. Switch between different tmux sessions.

### 3.3. Attaching:
If your terminal closes, just log back in and type:
`tmux a`.
This "attaches" you back to your last session exactly where you left off.

---

## 4. Alias
In Linux, an alias is essentially a custom shortcut or a "nickname" for a command. It allows you to replace a long, complex, or hard-to-remember command with a short, simple word of your choosing.

Open ~/.bashrc, scroll to the very bottom of the file and paste this line:

```bash
alias cleanros='rm -rf ~/.ros/log/*'
```

Apply the changes:
```bash
source ~/.bashrc
```

```info
The VM and Robot is setup with a bunch of useful aliases already. Type 'alias' to see a list of aliases already configured and ready to use in your system. 
```

### How it Works: The "Current Shell" Difference
`source ~/.bashrc` tells your current terminal window to "re-read" its configuration file and apply any changes you just made immediately. Without this command, you would have to close your terminal and open a new one to see your new aliases or environment variables.

To understand why we use source, you have to understand how Linux runs scripts:
- **Normal Execution (./script.sh or bash script.sh)**: This creates a subshell (a new, temporary process). Any changes made inside that script—like a new alias or a variable—stay inside that temporary process and vanish the moment the script finishes.

- **Sourcing (source script.sh or . script.sh)**: This executes the commands inside your current shell. It’s like you are manually typing every line of the file directly into your prompt. Because it happens in your current session, the changes "stick.

---
## 5. Other Useful CLI Tools
The VM also has a bunch of CLI tools that makes dev lives easier. 

### 5.1. fd-find (`fd`)
`fd` is a simple, fast, and user-friendly alternative to the standard `find` command.

- **Quick Start:** `fd my_node`
- **Why it's better:** It ignores `.git` folders by default, uses colors, and is significantly faster than the built-in search.
- **Note:** On some systems, the command is `fdfind`.

---

### 5.2. ripgrep (`rg`)
`rg` is the world's fastest tool for searching **inside** files for specific text.

- **Quick Start:** `rg "Namespace"`
- **Why it's better:** If you have thousands of lines of code in your ROS workspace, `rg` will find the exact line of code you're looking for almost instantly. It respects your `.gitignore` so it won't search through build logs.

---

### 5.3. autojump (`j`)
`autojump` learns your habits so you can navigate your filesystem faster.

- **Quick Start:** `j my_robot_pkg`
- **How it works (Frecency):** It uses a "Frecency" algorithm (Frequency + Recency). It tracks which folders you visit most often and most recently. 
- Instead of typing `cd ~/colcon_ws/src/my_packages/subfolder/robot_logic`, you just type `j robot` and it "jumps" there.

---

## 6. References
- [fd-find GitHub](https://github.com/sharkdp/fd)
- [ripgrep (rg) GitHub](https://github.com/BurntSushi/ripgrep)
- [autojump (j) Documentation](https://github.com/wting/autojump)
- [tmux Wiki](https://github.com/tmux/tmux/wiki)
