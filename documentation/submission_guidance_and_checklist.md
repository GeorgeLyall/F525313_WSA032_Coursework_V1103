# 25WSA032 Submission Checklist

**Deadline: Friday 15 May 2026, 3:00 PM**

Submit via the **25WSA032 Coursework Submission Point** on Learn.

> Save this file to your `documentation/` folder so it is part of your committed repo.

---

## What you are submitting

You submit a **zip of your `.git` folder only**. This folder is the repository — it contains your entire version-controlled project history. The marker uses it to reconstruct your project. Your source files do not need to be included separately; they are already inside `.git` through your commit history.

> This is not a zip of your whole project folder, and not a zip of your source files directly. It is specifically the hidden `.git` folder inside your project folder. The `.git` folder *is* your repo.

---

## Check your folder structure and key files

Your project folder must be named with your student ID prefix:

```
B123456_25WSA032_Coursework/
├── .git/                              ← this is what you will zip and submit
├── arduino/
│   └── temperature_optimisation.ino  ← Task 2 — exact filename required
├── documentation/
│   └── (your documentation files)
├── robots/
│   └── robot_optimisation.py         ← Task 3 — exact filename required
└── README.md                         ← should contain meaningful content
```

### Task 2 and Task 3 — exact filenames

Incorrect names or locations may result in zero marks for that task.

| Task | File | Location |
|------|------|----------|
| Task 2 | `temperature_optimisation.ino` | `arduino/` folder |
| Task 3 | `robot_optimisation.py` | `robots/` folder |

### README.md

Your `README.md` at the root of your project should contain a meaningful description of your project — not the placeholder text from the initial setup. This contributes to your Task 1 documentation marks.

### Task 4 deliverables

Ensure all of the following are committed somewhere in your repo:

- Data file(s) — CSV or Excel — from your recording session
- Python script that reads the data and produces plots
- Saved figure files (e.g. PNG)
- Written discussion of your results — a markdown file in `documentation/` works well for this

---

## Make your final commit

Ensure all work is saved and committed before you zip. If you have been using Git tags to mark milestones (Task 1), add a final tag such as `submission` to mark this as your submission point.

---

## Finding and zipping your `.git` folder

### Show hidden files on Windows

The `.git` folder is hidden by default.

**Windows 11:** File Explorer → **View** menu → **Show** → tick **Hidden items**

**Windows 10:** File Explorer → **View** tab → tick **Hidden items**

### Zip it

1. Navigate into your `B123456_25WSA032_Coursework` folder
2. Right-click the `.git` folder
3. **Windows 11:** Compress to ZIP file &nbsp; | &nbsp; **Windows 10:** Send to → Compressed (zipped) folder
4. Name the zip: `B123456_25WSA032_Coursework.zip`

---

## If your `.git` folder is in the wrong place

If VS Code was opened on a parent folder when the repo was first initialised, your `.git` may be one level above your project folder.

**To check**, open a terminal and run:

```bash
git rev-parse --show-toplevel
```

- Path ends in `B123456_25WSA032_Coursework` — you are fine, proceed as normal.
- Path is a parent folder — navigate to that folder in File Explorer. The `.git` folder will be there. Zip and submit that one. Your history is intact.

---

## If you used GitHub

Push your final commit to GitHub first, then:

1. Go to your repository on GitHub
2. Click the green **Code** button → **Download ZIP**
3. Extract the downloaded ZIP
4. Locate the `.git` folder inside (make hidden files visible as above)
5. Zip that `.git` folder and submit it

---

## Submission readiness checks

- [ ] Folder named correctly: `B123456_25WSA032_Coursework`
- [ ] Final commit made with a clear message
- [ ] `arduino/temperature_optimisation.ino` committed
- [ ] `robots/robot_optimisation.py` committed
- [ ] Task 4 data, script, figures, and discussion all committed
- [ ] `README.md` contains meaningful content
- [ ] If using tags — a `submission` tag (or similar) is added
- [ ] You are zipping the `.git` folder — not the whole project folder, not just source files
- [ ] ZIP named `B123456_25WSA032_Coursework.zip`

---

## Common submission mistakes

| Mistake | Why it matters |
|---------|----------------|
| Zipping source files without `.git` inside | Commit history is missing — Task 1 cannot be assessed |
| Zipping the entire project folder rather than `.git` | Submit the `.git` folder only |
| Zipping before making the final commit | Commit first, then zip |
| Wrong filename for Task 2 or Task 3 | File may not be found during marking |
| `README.md` left as the placeholder | Missed documentation marks under Task 1 |

---

*Deadline: **Friday 15 May 2026, 3:00 PM** — 25WSA032 Coursework Submission Point on Learn*
