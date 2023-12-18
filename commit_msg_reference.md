# Conventional Commits

See https://www.conventionalcommits.org for details


#### Commit types description:

- `<type>!`:  Commit that introduces a breaking API change (correlating with MAJOR in Semantic
  Versioning)
- `feat`: A new feature
- `fix`: A bug fix
- `docs`: Documentation only changes
- `style`: Changes that do not affect the meaning of the code (white-space, formatting, missing
  semi-colons, etc)
- `refactor`: A code change that neither fixes a bug nor adds a feature
- `perf`: A code change that improves performance
- `test`: Adding missing tests or correcting existing tests
- `build`: Changes that affect the build system or external dependencies (example scopes: gulp,
  broccoli, npm)
- `ci`: Changes to our CI configuration files and scripts (example scopes: Travis, Circle,
  BrowserStack, SauceLabs)
- `chore`: Other changes that don't modify src or test files
- `revert`: Reverts a previous commit

#### Commit footer:

- `BREAKING CHANGE`: Commit that introduces a breaking API change (correlating with MAJOR in Semantic Versioning) 

#### Commit message structure
```
<type>[(<optional-scope>)]: <description>

[optional-body]

[optional-footer(s)]

```
