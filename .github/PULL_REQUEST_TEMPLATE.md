## Description

<!-- Describe what this PR changes. -->

## Review Procedure

<!-- Explain how to review this PR. -->

## Remarks

<!-- Write remarks as you like if you need them. -->

## Pre-Review Checklist for the PR Author

**PR Author should check the checkboxes below when creating the PR.**

- [ ] Read [pull request guidelines][pull-request-guidelines]
- [ ] Code follows [coding guidelines][coding-guidelines]
- [ ] Assign PR to reviewer

## Checklist for the PR Reviewer

**Reviewers should check the checkboxes below before approval.**

- [ ] Commits are properly organized and messages are according to the guideline
- [ ] Code follows [coding guidelines][coding-guidelines]
- [ ] (Optional) Unit tests have been written for new behavior
- [ ] PR title describes the changes

## Post-Review Checklist for the PR Author

**PR Author should check the checkboxes below before merging.**

- [ ] All open points are addressed and tracked via issues or tickets
- [ ] Write [release notes][release-notes]

## CI Checks

- **Build and test for PR / build-and-test-pr**: Required to pass before the merge.
- **Build and test for PR / clang-tidy-pr**: NOT required to pass before the merge. It is up to the reviewer(s). Found false positives? See the [guidelines][clang-tidy-guidelines].
- **Check spelling**: NOT required to pass before the merge. It is up to the reviewer(s). See [here][spell-check-dict] if you want to add some words to the spell check dictionary.

[clang-tidy-guidelines]: https://tier4.github.io/autoware.proj/tree/main/developer_guide/ClangTidyGuideline/
[coding-guidelines]: https://tier4.atlassian.net/wiki/spaces/AIP/pages/1194394777/T4
[pull-request-guidelines]: https://tier4.github.io/autoware.proj/tree/main/developer_guide/PullRequestGuideline/
[release-notes]: https://tier4.atlassian.net/wiki/spaces/AIP/pages/563774416
[spell-check-dict]: https://github.com/tier4/autoware-spell-check-dict#how-to-contribute
