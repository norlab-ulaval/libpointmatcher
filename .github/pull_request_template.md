# Description

### Summary:

### Changes and type of changes (quick overview):

-
-
-

---

# Checklist:

### Code related

- [ ] I have made corresponding changes to the documentation 
      (i.e.: function, class, script header, README.md)
- [ ] I have commented hard-to-understand code
- [ ] I have added tests that prove my fix is effective or that my feature works
- [ ] All tests pass locally with my changes 
      (Check [README.md #Contributing](https://github.com/norlab-ulaval/libpointmatcher/tree/develop#contributing)
      for local testing procedure using _libpointmatcher-build-system_)

### PR creation related

- [ ] My pull request `base ref` branch is set to the `develop` branch 
     (the _build-system_ won't be triggered otherwise)
- [ ] My pull request branch is up-to-date with the `develop` branch 
     (the _build-system_ will reject it otherwise)

### PR description related

- [ ] I have included a quick summary of the changes
- [ ] I have indicated the related issue's id with `# <issue-id>` if changes are of type `fix`
- [ ] I have included a high-level list of changes and their corresponding type
    - Types: `feat` `fix` `docs` `style` `refactor` `perf` `test` `build` `ci` `chore` `revert`
    - Breaking changes: `<type>!`
    - Reference:
      see [commit_msg_reference.md](https://github.com/norlab-ulaval/libpointmatcher/blob/develop/commit_msg_reference.md)
      in the repository root for details

---

## Note for repository admins

### Release PR related

- Only repository admins have the privilege to `push/merge` on the default branch (ie: `master`)
  and the `release` branch.
- Keep PR in `draft` mode until all the release reviewers are ready to push the release.
- Once a PR from `release` -> `master` branch is created (not in draft mode),
    - it triggers the _build-system_ test
    - (in-progress) and it triggers the _semantic release automation_
