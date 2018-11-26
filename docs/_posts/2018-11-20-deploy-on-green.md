# Deploy on green

When [Jimmy] started to work as a developer he got interrested in CI/CD, and methods
for that. Since this robot is going to be a reasonable small project he conviced
[Erik] that it should contain a small bit of this as well. ROS has a comunity
that embraces [CI] and we build upon them and will use the Travis integration
with GitHub.

## Git workflow

We do all the developement in separate feature branches and master is always
deployable. Our GitHub [repo] don't allow
pushing to master so we have to do all the merging on GitHub.

## Tests

When we write the tests the aim is to write tests first and code later, but only
if we have the structure somewhat set. We won't write the tests first when we
tinker around.

## Testing

We are using the Travis<->GitHub integration, and we do both code linting and
unit tests in separate steps. Integration tests and functional tests are going
to be manual steps, and above that we are going to need to manueal tests to
see that the robot behaves as expected.

## CD

We plan to do continous delivery and let the robot update when all tests passes,
aka Deploy on green. This will include downloading and building all code, both
the Arduino code and the code for the Raspberry pi.

[![Build Status](https://travis-ci.com/erik78se/robojoy.svg?branch=master)](https://travis-ci.com/erik78se/robojoy.svg?branch=master)

[Jimmy]: <https://github.com/HeMan>
[Erik]: <https://github.com/erik78se>

[CI]: <http://wiki.ros.org/CIs>

[repo]: <https://github.com/erik78se/robojoy>
