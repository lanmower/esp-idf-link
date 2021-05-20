# esp-idf-ci-action

GitHub Action for ESP32 CI

## Example repo with GitHub Action

- [georgik/esp-dev-kits](https://github.com/georgik/esp-dev-kits/blob/feature/github_action_build/.github/workflows/build-smart-panel.yml)
- [georgik/ctag-tbd](https://github.com/georgik/ctag-tbd/blob/master/.github/workflows/draft-release.yml)

## Usage

Workflow definition

```
jobs:
  build:

    runs-on: ubuntu-latest

    steps:
    - name: Checkout repo
      uses: actions/checkout@v2
      with:
        submodules: 'recursive'
    - name: esp-idf build
      uses: georgik/esp-idf-ci-action@latest
      with:
        path: 'esp32-s2-hmi-devkit-1/examples/smart-panel'
```

## How to specify custom version of ESP-IDF

GitHub does not support the specification of the Docker image tag as a variable.
It's the official limitation. To select the specific version of ESP-IDF you
can reference specific version of GitHub action:

```
uses: georgik/esp-idf-ci-action@latest
uses: georgik/esp-idf-ci-action@4.2.1
uses: georgik/esp-idf-ci-action@release-v4.3
```

Each of the tag marks corresponding branch of this repository.
