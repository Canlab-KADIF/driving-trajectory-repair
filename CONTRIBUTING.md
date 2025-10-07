# Contributing | 기여 안내

Thank you for your interest. This repository is a research artifact of a Korean
national R&D project (RS-2023-00232046) released as open-source software under
the project's public-SW obligation.

본 저장소는 국가연구개발과제(RS-2023-00232046)의 공개SW 의무에 따라 공개된 연구
산출물입니다.

## Before you start | 시작 전

Please open an issue describing the problem or proposal before sending a large
pull request. Small fixes (typos, build breakage, documentation) can go straight
to a pull request.

큰 변경은 먼저 이슈로 논의해 주십시오. 오탈자·빌드 오류·문서 수정 등 작은 변경은
바로 PR로 보내주셔도 됩니다.

## Development setup | 개발 환경

See the *Build* section of `README.md`. Note that this package depends on ROS
message definitions that are not publicly distributed; the `README.md` documents
their field layout so you can define equivalents locally.

## Code style | 코드 스타일

C++ sources:

- C++14, Google C++ Style Guide as the base
- `clang-format` with the repository `.clang-format` (Google, 80 columns) is
  authoritative — run it before committing
- headers use `.h`, sources use `.cc`, and every header starts with `#pragma once`
- methods are `PascalCase`, member variables are `snake_case_` with a trailing
  underscore, constants are `kPascalCase`
- prefer `const&` for inputs and a pointer for outputs
- recoverable failures return `bool`; do not throw for control flow

Python sources:

- PEP 8, four-space indentation, type hints on public functions
- module and function names in `snake_case`, classes in `PascalCase`

All sources:

- every file must start with the Apache-2.0 license header (see any existing file)
- no trailing whitespace
- comments in English

```bash
# format C++ before committing
find src -name '*.h' -o -name '*.cc' | xargs clang-format -i
```

## Commit messages | 커밋 메시지

Use imperative mood and a short scope prefix:

```
bag_modifier: replace CGAL polygon intersection with an internal implementation
docs: document the PerceptionObstacle message layout
```

## Pull requests | 풀 리퀘스트

Before requesting review, make sure:

- `scripts/check_release_ready.sh` passes
- the package builds (`catkin_make`)
- new behaviour is covered by a test where practical
- documentation is updated when interfaces change

## Licensing of contributions | 기여물의 라이선스

By submitting a contribution you agree that it is licensed under the
Apache License, Version 2.0, consistent with the rest of this repository
(see `LICENSE`, and Section 5 of the license regarding contributions).

기여하신 코드는 본 저장소와 동일하게 Apache License 2.0으로 배포됩니다.
