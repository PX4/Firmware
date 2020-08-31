# Contributing to PX4 Firmware

We follow the [Github flow](https://guides.github.com/introduction/flow/) development model.

### Fork the project, then clone your repo

First [fork and clone](https://help.github.com/articles/fork-a-repo) the project project.

### Create a feature branch

*Always* branch off master for new features.

```
git checkout -b mydescriptivebranchname
```

### Edit and build the code

The [developer guide](http://dev.px4.io/) explains how to set up the development environment on Mac OS, Linux or Windows. Please take note of our [coding style](https://dev.px4.io/master/en/contribute/code.html) when editing files.

### Commit your changes

Always write descriptive commit messages and add a fixes or relates note to them with an [issue number](https://github.com/px4/Firmware/issues) (Github will link these then conveniently)

**Example:**

```
Change how the attitude controller works

- Fixes rate feed forward
- Allows a local body rate override

Fixes issue #123
```

### Test your changes

Since we care about safety, we will regularly ask you for test results. Best is to do a test flight (or bench test where it applies) and upload the logfile from it (on the microSD card in the logs directory) to Google Drive or Dropbox and share the link.

### Push your changes

Push changes to your repo and send a [pull request](https://github.com/PX4/Firmware/compare/).

Make sure to provide some testing feedback and if possible the link to a flight log file. Upload flight log files to [Flight Review](http://logs.px4.io) and link the resulting report.
