[CmdletBinding()]
param (
    [Parameter(Mandatory=$true,
    HelpMessage="Specify ESP-IDF tag name to create branch and tag of the action, tags should match https://hub.docker.com/r/espressif/idf/tags")]
    [String]
    $IdfTag
)

git checkout master
git checkout -b "${IdfTag}"
sed -i "1s!.*!FROM espressif/idf:${IdfTag}!" Dockerfile
git add Dockerfile
git commit -m "tools: add GitHub action for ESP-IDF ${IdfTag}"

# Note: It's sufficient to define branch for GitHub action to pick correct @ref.
# Having branch and tag name same leads to ambiguous ref
#git tag "${IdfTag}"

git push --set-upstream origin "${IdfTag}"
git push
git checkout master
