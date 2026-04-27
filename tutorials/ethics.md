---
layout: default
title: "Ethics"
parent: Tutorials
sort: 13
---

# Handout: Formal Methods for Ethical Engineering

## Part 1: Ethical Frameworks

### 1. Levels of Ethical Analysis
Engineering ethics operates at two distinct scales. Understanding the level of a problem helps determine the appropriate intervention and which formal test to prioritize.

* **Micro-Ethics**: Concerned with individuals and the internal relations of the engineering profession. It focuses on personal conduct, individual responsibility, and the "whistleblowing" dilemmas faced by engineers (e.g., *Should I write this specific line of code?*).
* **Macro-Ethics**: Concerned with the collective, social responsibility of the engineering profession and societal decisions about technology. It asks how a technology impacts the world at large (e.g., *How does autonomous transit change urban poverty?*).

---

### 2. The Formal Methods (The Three Tests)
When evaluating an action, engineers apply these formal tests to move beyond "gut feelings" toward rigorous, defensible analysis.

#### A. The Utilitarian Test (Best Outcomes)
This theory determines right from wrong by focusing on the consequences of an action. It is often the primary tool for **Macro-ethical** analysis.

* **Core Principle**: The most ethical choice is the one that produces the **greatest good for the greatest number**.
* **Application Steps**:
    1.  **Identify Alternatives**: List possible actions and all current/future stakeholders.
    2.  **Cost-Benefit Analysis**: Determine the benefits and costs for all affected parties, predicting outcomes based on facts.
    3.  **Selection**: Choose the action producing the greatest net benefits.
    4.  **Universal Policy**: Ask what would happen if this action became a universal standard.
* **Strength**: Provides a rational, "mathematical" approach to decision-making.
* **Weakness**: It is difficult to predict all future probabilities in complex robotics systems.

#### B. The Justice Test (Fairness)
This test focuses on how benefits and burdens are distributed across a population. It is critical for identifying algorithmic bias.

* **Core Principle**: Equals should be treated equally. Inequality (e.g., higher error rates for one group) must be justified by valid criteria like effort or need, not protected characteristics.
* **Application Steps**:
    1.  **Analyze Distribution**: Determine who receives the benefits and who bears the burdens of the technology.
    2.  **Assess Fairness**: Determine if those who get the benefits also share the risks proportionally.
    3.  **Conflict Resolution**: If fairness is disputed, select a fair process (like a vote or third-party audit) to decide.
* **Strength**: Protects vulnerable groups and ensures long-term social stability.
* **Weakness**: There is rarely a universally agreed-upon definition of "perfectly fair."

#### C. The Virtue Test (Character)
This test focuses on the integrity of the actor and professional standards. It is the core tool for **Micro-ethical** dilemmas.

* **Core Principle**: Asks if an action represents the kind of person you want to be or the reputation your profession seeks to uphold.
* **Application Steps**:
    1.  **Role Model Check**: Is this what the most respected person in your field would do?
    2.  **Vision Alignment**: Does this fit the company’s long-term reputation and your own integrity?
    3.  **Professional Balance**: Does the action maintain a balance between technical excellence and success?
* **Strength**: Emphasizes that ethics is a "habit" developed through character.
* **Weakness**: Human behavior can be inconsistent when facing extreme external pressure (like the VW scandal).

---

## Part 2: Case Studies for Today

### Case 1: Uber Phoenix Accident
A self-driving car hit a pedestrian because the software was tuned to ignore "false positives" (objects like plastic bags) to ensure a smooth ride.
* **Audit Goal**: Contrast the **Utility** of a "smooth ride" for the passenger vs. the **Justice** of safety for the unconsenting pedestrian.

### Case 2: COMPAS Recidivism Tool
Algorithms used to predict re-offending rates were found to flag Black defendants as "high risk" twice as often as white defendants, despite similar actual re-offense rates.
* **Audit Goal**: Evaluate if the system fails the **Justice Test** by placing a disproportionate burden on a specific group.

### Case 3: Amazon Recruiting Tool
AI screened resumes but penalized candidates from "Women’s colleges" because it was trained on a decade of male-dominated hiring data.
* **Audit Goal**: Does an engineer's **Virtue** require them to "fix" the data, or simply "report" what the data says?

### Case 4: The Brake Failure Dilemma
A car's brakes fail. It must choose: 1) Hit a pedestrian on a crosswalk, or 2) Swerve into a pole and harm the driver.
* **Audit Goal**: Perform a **Utilitarian** calculation. Does the number of people involved change the "correct" code?

### Case 5: Generative Image Deepfakes
Software creates hyperrealistic images of people without any visible or invisible watermarks to maximize "marketability."
* **Audit Goal**: Analyze the **Macro-ethical** impact on society’s ability to trust visual evidence.

---

## Part 3: Presentation Scorecard
Your team must evaluate your assigned case and be prepared to defend your reasoning to the class. Start your presentation with the case study description and audit goal.

## Part 3: Presentation & Hearing Instructions

Each team will act as an **Engineering Ethics Board**. You have 3 minutes to present your findings, followed by a 3-minute "Hearing" (Q&A).

### 1. Presentation Requirements (The 3-Minute Pitch)
Focus on the analytical "Ethics Scorecard", Audit Goal, and your Engineering fix:
> You may choose to use the internet to find more information about the case.

- **Ethics Scorecard**: State clearly which tests your case **Passed** or **Failed** (Utilitarian, Justice, Virtue), and for every "Fail," provide the specific technical reason.

    | Test | Status | Reasoning (Cite Part 1 Definitions) |
    | :--- | :--- | :--- |
    | **Utilitarian Test** | Pass / Fail | |
    | **Justice Test** | Pass / Fail | |
    | **Virtue Test** | Pass / Fail | |

- **Audit Goal:** Discuss the audit goal for the assigned case.
- **Engineering Fix**: Propose one specific technical change to the system (e.g., a software lock, a mandatory metadata watermark, or a sensor redundancy requirement) that would allow the project to pass one of failed tests. 

### 2. The "Hearing" (The 3-Minute Q&A)
Following your presentation, the floor will open for cross-examination.

- When a team presents, the previous team that presented must ask the first question.
- All questions and answers must use the formal terminology from Part 1.(e.g., How does your 'Engineering Fix' satisfy the **Virtue Test**...?)

---
