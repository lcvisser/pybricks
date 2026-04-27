# PyBricks development

Required dependencies are `pybricks` and `pybricksdev`.

Add the following launch configuration:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Run on hub",
            "type": "debugpy",
            "request": "launch",
            "module": "pybricksdev",
            "args": ["run", "ble", "${file}"]
        }
    ]
}
```

More information: https://pybricks.com/project/pybricks-other-editors/
