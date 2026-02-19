---
layout: default
title: "Github Pages"
parent: Tutorials
sort: 7
---
# Github Pages for Project

## 1. Getting Started
The [repository](https://github.com/ras-mobile-robotics/project-site-template) uses **Jekyll**, a static site generator, to turn your **Markdown** files into a professional website. You don't need to write HTML or CSS; you only need to focus on your content.

Setting up the project requires two primary steps: initializing the site and adding your teammates.

```info
For a quick introduction to git, checkout the [git tutorial](../tutorials/git.md).
```

### 1.1 Create the Repository

> **Note:** Only **one person** in the group needs to perform this step.

1. Click **"Use this template"** (top right corner) > **Create a new repository**.
2. Navigate to **Settings** > **Pages** (in the sidebar).
3. Under **Build and deployment**, change the **Source** dropdown to **GitHub Actions**.

#### **First-Time Deployment Fix**

The first build often fails because permissions need to be initialized. Follow these steps to fix it:

1. Go to the main page of your repository.
2. Look at the icon next to your most recent commit (near the top of the file list):
* 🟡 **Yellow Circle:** Jekyll is building. Wait 1–2 minutes.
* ✅ **Green Checkmark:** Success! Your site is live.
* ❌ **Red X:** The build failed. **Do the following:**
* Click the ❌ to open the log.
* Click **Details**.
* Click the **Re-run all jobs** button at the top right.
* Return to the main page and wait for the ✅.

To find your live website link, see [How to Find the Deployed Link](#52-how-to-find-the-deployed-link). 

---

### 1.2 Add Team Members

Once the repo is created, invite your teammates to give them push access.

1. In your repository, click the **Settings** gear icon.
2. Select **Collaborators** from the left-hand sidebar (under the *Access* category).
3. Click the green **Add people** button.
4. Search for your teammate's **GitHub username** or email and click **Add to this repository**.

> **Important: Acceptance Required**
> Teammates are not added automatically. They must accept the invite via the link sent to their **email**.

#### **Recommended Permissions**

* **Write:** (Default) Standard for all members. Allows pushing code and creating branches.
* **Admin:** Optional for the Team Lead. Allows managing repository settings and members.

---

## 2. Understanding the Tech Stack

### 2.1. What is Markdown?

Markdown is a lightweight markup language with plain-text formatting syntax. It allows you to write using easy-to-read symbols (like `#` for headings or `**` for bold) that are later converted into high-quality HTML.

### 2.2. What is Jekyll?

Jekyll is the "engine" that powers this site. When you upload a Markdown file, Jekyll reads your **Front Matter** (the configuration at the top of the file), processes your text, and wraps it in a consistent theme (**Just the Docs**).

### 2.3. How it works together

1. **You write** a `.md` file in GitHub.
2. **You Push** your changes to the repository.
3. **GitHub Actions** detects the change and automatically runs Jekyll to build your site.
4. **GitHub Pages** hosts the resulting website for the world to see.

---

## 3. Page Organization

Your workspace is pre-organized with a directory named **Project**, located in the `/project/` folder.
You may add more folders if you want.

### 3.1 The Header (Front Matter)

Every file must start with this exact block. It tells the sidebar where the page belongs:

```markdown
---
layout: default
title: "Report 1"
parent: Project
nav_order: 1
---
```

* **title:** The name shown in the sidebar.
* **parent:** Must match the category folder exactly (`Projects`).
* **nav_order:** Determines the sequence (1, 2, 3...) in the sidebar.

---

## 4. Customizing the Homepage (`index.md`)

The `index.md` file in the root directory is your site’s landing page. You should edit this to introduce the team, its inspiration, its members and your work. This file serves as the entry point for your entire site architecture.

**Best Practices for Content:**

- **Introduction:** Replace the template text with a brief professional bio of your team mates, a description of what this repository contains.
- **Visuals:** This is the best place to add profile pictures or a banner image related to your robotics project.

---

## 5. Checking Your Build Status

After you **"Commit"** and **"Push"** your changes to GitHub, the website does not update instantly. It takes about 30–60 seconds to "build."

### 5.1. How to check if it worked?

1. Go to the main page of your **GitHub Repository**.
2. Look next to your most recent commit message (near the top of the file list).
3. You will see a **small icon**:
* 🟡 **Yellow Circle:** Jekyll is currently building your site. Wait a moment.
* ✅ **Green Checkmark:** Success! Your changes are now live on the website.
* ❌ **Red X:** The build failed. This usually means there is a typo in your **Front Matter** (like a missing quote or incorrect indentation). Click the X to see the error log.


### 5.2. How to find the deployed link?

1. Go to the main page of your GitHub repository.
2. Look at the right-hand sidebar.
3. Under the **Deployments** or **Environments** section, click on **github-pages**.
4. The page should show the URL of your GitHub Pages website and the deployment history.

---

## 6. Formatting Reference

For a complete list of everything you can do (tables, callouts, buttons, and advanced lists), open the **`markdown_kitchen_sink.md`** file in this repository.

#### Quick Tips:

* **Math:** Use LaTeX for formulas. Use `$$...$$` for standalone block equations and `$ ... $` for inline math.
* **Code:** Wrap code in triple backticks and specify the language: ````python`.
* **Callouts:** Use `{: .note }` or `{: .warning }` immediately after a blockquote to highlight important info.

> **Note**
> This is a callout box for important information.
> {: .note }

---

## 7. More Resources

If you want to further customize your site or troubleshoot complex layouts, check out the links below. This is **strictly optional**.

* **[Just the Docs Documentation](https://just-the-docs.com/):** The official guide for the theme used in this repository. It includes specific instructions for navigation, UI components, and color schemes.
* **[Jekyll Step-by-Step Tutorial](https://jekyllrb.com/docs/step-by-step/01-setup/):** A deep dive into how Jekyll generates sites from scratch.
* **[GitHub Actions Documentation](https://docs.github.com/en/actions):** Official GitHub help to automate, customize, and execute your software development workflows.
* **[Liquid Syntax Reference](https://current-rms.gitbook.io/liquid-syntax):** Jekyll uses Liquid as its templating language. This is useful if you want to create complex logic like "if/else" statements or loops within your Markdown.
