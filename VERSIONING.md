# Git Versioning Guide

This repository now uses Git as the single source of truth for history.

## Current stable history

- `master` currently points to `eadc886` (`220 两圈不带圆环`).
- `0d6dc42` is the earlier `260` snapshot.
- `8eaf178` is the `串级240，校赛` snapshot.

## Rules

1. `main_original` should always mean the untouched baseline.
2. `main_master` should always mean the working integration branch.
3. Every major change should be captured by a commit before the next risky edit.
4. Every stable milestone should get an annotated tag.
5. Build outputs in `MDK/Out_File/` are ignored and must not be used as history.

## Suggested tag format

- `snap-YYYYMMDD-HHMM-<module>`
- `release-YYYYMMDD`
- `baseline-<short-description>`

## Recovery commands

```bash
git log --oneline --graph --decorate --all
git branch -a
git tag
git switch -c rescue/<date>
```

## Notes

- If a generated file is still tracked, it should only be removed from Git after confirming it is not needed for release history.
- Current uncommitted source changes are still in the working tree until they are committed or stashed.

