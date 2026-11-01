# Authors whose pull requests get a full kernel build on every push.
#
# The kernel team. A pull request from anyone listed here builds the package
# and runs KUnit and the boot test on every push once it is out of draft,
# from a fork too, with no ci:build label needed, as long as whoever pushed
# is listed too. Everyone else, omacom
# organization members and repository collaborators included, gets the
# checks only until a maintainer applies ci:build to that pull request. CI
# never builds a denounced author's pull requests, label or not.
#
# CI reads this file from the target branch, so a pull request cannot vouch
# for its own author. GitHub holds pull request runs from outside
# collaborators; kernel-ci-approve.yml releases them for those listed here.
#
# Syntax (the mitchellh/vouch format omacom/omarchy-pkgs uses):
#   github:username
#   -github:username reason for denouncement
#
# Keep entries sorted alphabetically, ignoring case.
github:Acelogic
github:chriskearney
github:dhh
github:DjDeveloperr
github:emirb
github:erykwieliczko
github:joshuaswarren
github:kwilczynski
github:LiamRay11
github:malik-na
github:maralcbr
github:outfoxxed
github:ryanrhughes
github:RyanTheTide
github:satchlj
github:scottjones
github:themartiano
