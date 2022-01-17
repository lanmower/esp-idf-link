[CmdletBinding()]
param (
    [Parameter(Mandatory = $true,
        HelpMessage = "Specify ESP-IDF tag name to create branch and tag of the action, tags should match https://hub.docker.com/r/espressif/idf/tags")]
    [String]
    $IdfTag
)

$IdfBranch = "branch-${IdfTag}"

git checkout master
git checkout -b "${IdfBranch}"
sed -i "1s!.*!FROM espressif/idf:${IdfTag}!" Dockerfile
git add Dockerfile
git commit -m "tools: add GitHub action for ESP-IDF ${IdfBranch}"
git tag "${IdfTag}"
git push --set-upstream origin "${IdfBranch}"
git push
git push --tags
git checkout main
