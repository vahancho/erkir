# Scripts

> **Note**: Povide _execute_ permission with `chmod u+x <SCRIPT>`

## [`check.sh`](./check.sh)

Check errors

```sh
./check.sh
```

### Arguments

> **Note**: Type `--help` for more information

| **Name**              | **Description**                                                                                                                                                      | **Default**      |
| --------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------- |
| `--disable-color`     | Disable color                                                                                                                                                        | `false`          |
| `--help`              | Show help message and exit                                                                                                                                           |
| `--log-level <LEVEL>` | Logger level <br/> <ul><li>`error`<br/>Error level</li><li>`warn`<br/>Warning level</li><li>`info`<br/>Informational level</li><li>`debug`<br/>Debug level</li></ul> | `info`           |
| `--root-dir <DIR>`    | Root directory                                                                                                                                                       | `[SCRIPTDIR]/..` |
