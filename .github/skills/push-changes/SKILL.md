---
name: push-changes
description: "Use when: committing and pushing code changes to GitHub with a commit message that summarizes the changes. Keywords: commit, push, git, GitHub, publish changes, save to GitHub."
argument-hint: "[optional commit message hint]"
---

# Commit and push changes to GitHub

Repository root: `C:\Users\jan\Documents\Elektronik\ThePreamp` (remote `origin`, Gninnorg/ThePreamp). Always use `git --no-pager`.

## Steps

1. Run `git --no-pager status --short` and `git --no-pager branch --show-current`. If there are no changes, say so and stop.
2. Review the changes with `git --no-pager diff HEAD --stat` and `git --no-pager diff HEAD` (also look at untracked files from status).
3. Do not commit build output or secrets (`.pio/`, `.env`, credentials, `*.bin` build artifacts). If any such file is untracked or modified, exclude it and tell the user.
4. Update the README before committing, so documentation and code are pushed together:
   - Read `Software\ThePreamp Controller\README.md` and compare it with the diff from step 2.
   - Edit only the sections the changes affect (behavior, endpoints, settings, hardware/pins, build/flash/test instructions, file names). Keep the existing structure and style, and don't rewrite unrelated parts.
   - If nothing in the README is affected (e.g. a pure refactor, or only the README itself changed), leave it alone and say so in the report.
   - Include the README in the commit.
5. Stage the relevant files, including the README (`git add <paths>`; `git add -A` only if nothing needs excluding).
6. Write the commit message from the diff:
   - Subject line: imperative, at most 72 characters, summarizing the change (e.g. `Fix encoder debounce and update settings page`).
   - Optional body: short bullet list of the notable changes and why, when the change spans several areas.
   - Use the user's hint (argument) if provided.
   - End the message with the trailer: `Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>`
   - Pass multi-line messages with multiple `-m` flags (PowerShell has no heredoc).
7. Commit, then `git push` (use `git push -u origin <branch>` if the branch has no upstream).
8. If the push is rejected, run `git pull --rebase` and retry once; stop and report on conflicts. Never force-push.
9. Report the commit hash, subject, and branch pushed, and whether the README was updated.
