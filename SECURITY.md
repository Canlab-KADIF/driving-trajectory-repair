# Security Policy | 보안 정책

## Supported versions

Only the latest release on the default branch receives fixes. This software is a
research artifact produced by a national R&D project; it is **not** intended for
deployment on a production vehicle or in a safety-critical control loop.

기본 브랜치의 최신 릴리스만 수정을 제공합니다. 본 소프트웨어는 국가연구개발과제의
연구 산출물이며, 실차 양산 환경이나 안전 필수(safety-critical) 제어 루프에서의
사용을 전제로 하지 않습니다.

## Reporting a vulnerability | 취약점 신고

Please **do not** open a public GitHub issue for security problems.

Report privately to the maintainer contact listed in `package.xml`, including:

- affected version or commit hash
- reproduction steps
- observed and expected behaviour
- potential impact

We aim to acknowledge a report within 10 business days.

보안 문제는 공개 이슈로 등록하지 마시고, `package.xml`에 기재된 유지보수 연락처로
비공개 신고해 주십시오. 영향 버전/커밋, 재현 절차, 관찰된 동작과 기대 동작, 예상
영향 범위를 함께 보내주시면 검토 후 영업일 기준 10일 이내에 회신합니다.

## Scope | 신고 범위

In scope:

- code execution or file overwrite triggered by a malformed input bag file
- unsafe deserialization of recorded messages
- leakage of credentials or personally identifiable information in logs

Out of scope:

- vulnerabilities in ROS, the simulator, or other upstream dependencies
  (report those to their respective projects)
- issues that require an already-compromised host
