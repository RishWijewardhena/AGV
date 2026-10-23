# Commit message style

Use one atomic commit for each logical change. Keep unrelated files out of the commit.

## Format

```text
type(scope): short imperative summary

- first reason or change
- second reason or change
- third reason or change
```

- Put the subject on the first line; use lowercase after the colon and no final period.
- Leave one blank line after the subject.
- Write the body **only as bullet points**. Do not add a prose paragraph before the bullets.
- Keep each bullet specific to the files and behaviour in that commit.
- Use a meaningful scope, such as `agv_sim`, `agv_control`, `agv_bringup`, `project`, or a document name.

## Types

- `feat`: add functionality.
- `fix`: correct a bug.
- `docs`: change documentation only.
- `test`: add or update tests only.
- `refactor`: change code structure without changing behaviour.
- `chore`: maintenance, tooling, or configuration.

## Example

```text
feat(agv_control): stop drive commands near obstacles

- gate raw velocity commands using the four distance readings
- hold the robot when readings are blocked or stale
- resume only after the path stays clear
```

For a small change, one or two bullets are enough. Split a change into separate commits when its parts can be reviewed or reverted independently.
