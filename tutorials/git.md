---
layout: default
title: "git"
parent: Tutorials
sort: 6
---

## The Git Guide: From Kernel to Cloud

### 1. A Brief History

In 1991, **Linus Torvalds** created **Linux**. For years, the Linux team managed code by passing manual "patches" around. In 2005, when their existing version control tool (BitKeeper) was no longer free, Linus took a weekend to write **Git**. He built it to be **distributed** (everyone has a full copy), **fast**, and **secure** against data corruption.

---

### 2. The Core Workflow

<p align="center">
  <img src="https://media2.dev.to/dynamic/image/width=800%2Cheight=%2Cfit=scale-down%2Cgravity=auto%2Cformat=auto/https%3A%2F%2Fdev-to-uploads.s3.amazonaws.com%2Fuploads%2Farticles%2Fvpxeexqyfvf4hw3zxtbn.png" width="600" alt="git workflow">
</p>

Git moves data between four distinct stages. Refer to your diagram to see how these commands act as the "transport" system:

* **Workspace (Working Directory):** This is where you actually edit files. Git sees these as "Untracked" or "Modified."
* **Staging Area (Index):** A "pre-commit" zone. You use `git add` to tell Git, "I want these specific changes to be in my next snapshot."
* **Local Repository:** This is on your computer. When you `git commit`, Git saves a permanent snapshot of your staged files here.
* **Remote Repository:** Hosted on the internet (GitHub). You `git push` to share your local work or `git pull` to grab others' work.

---

### 3. Setup & Configuration


#### 3.1. Option A: VS Code (The "Visual" Way)

```note
If you are new to git, this is the recommended option to use git.
```

Perfect for beginners, VS Code provides a sidebar that handles the arrows in your diagram automatically.

Checkout this [VSCode tutorial](https://code.visualstudio.com/docs/sourcecontrol/overview) for using git within VSCode.

#### 3.2. Option B: CLI Tools (The "Power User" Way)

<p align="center">
  <img src="https://user-images.githubusercontent.com/14274827/91470661-9d5a8780-e8b2-11ea-9ccb-0d813d2e35d1.png" width="500" alt="git workflow">
</p>

The Command Line Interface (CLI) is how Git was originally intended to be used. It is fast and works on any server.

Checkout this [W3Schools tutorial](https://www.w3schools.com/git/) for using git directly in the terminal.

**A Shallow-Deep Dive: Moving Data**

* `git commit -a`: This bypasses the staging area for modified files, moving them directly from your Workspace to the Local Repo.
* `git checkout`: This pulls a file from your Local Repo/Staging and overwrites what is in your Workspace (useful for "undoing" mistakes).
* `git fetch` vs `git pull`: `fetch` only looks at what's new on the remote repo (GitHub/BitBucket/Codeberg); `pull` actually brings those changes into your files.

---

## 5. Git vs. GitHub: What's the Difference?

While they are often mentioned together, they serve very different purposes in your development workflow.

### 5.1. Git: The Engine (Local)

* **What it is:** A piece of software that runs on your computer.
* **Function:** It tracks the history of your files and handles the arrows on the left side of your diagram (Workspace → Index → Local Repo).
* **Requirement:** It does not require an internet connection to work. You can commit and branch entirely offline.

### 5.2. GitHub: The Hub (Remote)

* **What it is:** A cloud-based hosting service for Git repositories.
* **Function:** It acts as the "Remote Repository" on the right side of your diagram. It provides a web interface to view code, manage projects, and collaborate with others.
* **Extra Features:** GitHub adds social and management layers that Git doesn't have, such as:
* **Pull Requests:** A way to ask a project owner to "pull" your changes into their code.
* **Issues:** A built-in bug tracker and "to-do" list for the project.
* **GitHub Actions:** The automation tool you are using to deploy your Jekyll site.


### 5.3. How they interact

When you use the command `git push origin main`, you are telling the **Git** software on your computer to send your local snapshots up to the **GitHub** servers. This bridges the gap between your private work and the public (or shared) project.

---

## 6. Workflow Summary

| Step | Location | Tool Used | Command |
| --- | --- | --- | --- |
| **Write Code** | Workspace | VS Code | (Save file) |
| **Stage Changes** | Index | Git | `git add` |
| **Save Snapshot** | Local Repo | Git | `git commit` |
| **Share Work** | Remote Repo | **GitHub** | `git push` |