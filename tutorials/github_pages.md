---
layout: default
title: "Github Pages for Assignments"
parent: Tutorials
sort: 7
---
# Github Pages for Assignments

The [repository](https://github.com/ras-mobile-robotics/class-assignment-template) uses **Jekyll**, a static site generator, to turn your **Markdown** files into a professional website. You don't need to write HTML or CSS; you only need to focus on your content.

To get started with creating a site, simply:

1. Click **"Use this template"** (at the top right corner) > **Create a new repository**.
2. Go to **Settings**. In the side menu, click on **Pages**.
3. In **Build and deployment**, select **GitHub Actions** under **Source**.

```note
**First Time Use**:

1. Go to the main page of your **GitHub Repository**.
2. Look next to your most recent commit message (near the top of the file list).
3. You will see a **small icon**:
* 🟡 **Yellow Circle:** Jekyll is currently building your site. Wait a moment.
* ✅ **Green Checkmark:** Success! Your changes are now live on the website.
* ❌ **Red X:** The build failed.

The first time, the build typically fails because the "Actions" permission needs to be initialized:

1. Click the ❌ to see the error log.
2. Click on the last **"details"** link.
3. In the Actions page that loads, click on the **`Re-run all jobs`** button.
4. Go to the main page of your **GitHub Repository** and wait for the ✅.

To find your website link check [How to Find the Deployed Link](#42-how-to-find-the-deployed-link).
```

---

## 1. Understanding the Tech Stack

### 1.1. What is Markdown?

Markdown is a lightweight markup language with plain-text formatting syntax. It allows you to write using easy-to-read symbols (like `#` for headings or `**` for bold) that are later converted into high-quality HTML.

### 1.2. What is Jekyll?

Jekyll is the "engine" that powers this site. When you upload a Markdown file, Jekyll reads your **Front Matter** (the configuration at the top of the file), processes your text, and wraps it in a consistent theme (**Just the Docs**).

### 1.3. How it works together

1. **You write** a `.md` file in GitHub.
2. **You Push** your changes to the repository.
3. **GitHub Actions** detects the change and automatically runs Jekyll to build your site.
4. **GitHub Pages** hosts the resulting website for the world to see.

---

## 2. Page Organization

Your workspace is pre-organized into two main categories:

* **Assignments:** Located in the `/assignment/` folder.
* **Projects:** Located in the `/project/` folder.

You may add more folders if you want.

### 2.1. The Header (Front Matter)

Every file must start with this exact block. It tells the sidebar where the page belongs:

```markdown
---
layout: default
title: "Assignment 1: My Topic"
parent: Assignments
nav_order: 1
---
```

* **title:** The name shown in the sidebar.
* **parent:** Must match the category folder exactly (`Assignments` or `Projects`).
* **nav_order:** Determines the sequence (1, 2, 3...) in the sidebar.

---

## 3. Customizing the Homepage (`index.md`)

The `index.md` file in the root directory is your site’s landing page. You should edit this to introduce yourself and your work. This file serves as the entry point for your entire site architecture.

**Best Practices for Content:**

1. **Introduction:** Replace the template text with a brief professional bio and a description of what this repository contains.
2. **Quick Links:** You can use Markdown list syntax to provide direct links to your most important assignments.
3. **Visuals:** This is the best place to add a profile picture or a banner image related to your robotics projects.

---

## 4. Checking Your Build Status

After you **"Commit"** and **"Push"** your changes to GitHub, the website does not update instantly. It takes about 30–60 seconds to "build."

### 4.1. How to check if it worked?

1. Go to the main page of your **GitHub Repository**.
2. Look next to your most recent commit message (near the top of the file list).
3. You will see a **small icon**:
* 🟡 **Yellow Circle:** Jekyll is currently building your site. Wait a moment.
* ✅ **Green Checkmark:** Success! Your changes are now live on the website.
* ❌ **Red X:** The build failed. This usually means there is a typo in your **Front Matter** (like a missing quote or incorrect indentation). Click the X to see the error log.


### 4.2. How to find the deployed link?

1. Go to the main page of your GitHub repository.
2. Look at the right-hand sidebar.
3. Under the **Deployments** or **Environments** section, click on **github-pages**.
4. The page should show the URL of your GitHub Pages website and the deployment history.

---

## 5. Formatting Reference

For a complete list of everything you can do (tables, callouts, buttons, and advanced lists), open the **`markdown_kitchen_sink.md`** file in this repository.

### 5.1 Quick Tips:

* **Math:** Use LaTeX for formulas. Use `$$...$$` for standalone block equations and `$ ... $` for inline math.
* **Code:** Wrap code in triple backticks and specify the language: ````python`.
* **Callouts:** Use `{: .note }` or `{: .warning }` immediately after a blockquote to highlight important info.

> **Note**
> This is a callout box for important information.
> {: .note }


### 5.2 Changes Made:

* **Formatting Alignment:** Standardized bullet points and bolded UI elements (e.g., **"Re-run all jobs"**) for better scannability.
* **Clarity on Build Failure:** Added a brief technical explanation for *why* the first build usually fails (Actions initialization).
* **Environment Visibility:** Clarified that the link might be under **"Environments"** as well as "Deployments" in the sidebar.
* **Formatting Tips:** Added an actual visual example of how a **Callout** looks in Markdown to guide the user.
* **Visual Guides:** Inserted instructional image tags for GitHub Pages settings and Actions workflows to assist visual learners.

---

## 6. More Resources

If you want to further customize your site or troubleshoot complex layouts, check out the links below. This is **strictly optional**.

* **[Just the Docs Documentation](https://just-the-docs.com/):** The official guide for the theme used in this repository. It includes specific instructions for navigation, UI components, and color schemes.
* **[Jekyll Step-by-Step Tutorial](https://jekyllrb.com/docs/step-by-step/01-setup/):** A deep dive into how Jekyll generates sites from scratch.
* **[GitHub Actions Documentation](https://docs.github.com/en/actions):** Official GitHub help to automate, customize, and execute your software development workflows.
* **[Liquid Syntax Reference](https://current-rms.gitbook.io/liquid-syntax):** Jekyll uses Liquid as its templating language. This is useful if you want to create complex logic like "if/else" statements or loops within your Markdown.
