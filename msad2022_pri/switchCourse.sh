#!/usr/bin/env bash
usage_exit() {
        echo "Usage: $0 [-q]" 1>&2
        echo "  -q query current course setting" 1>&2
        echo "" 1>&2
        exit 1
}

PROFILE="msad2022_pri/profile.txt"
QUERY=""

while getopts qh OPT
do
    case $OPT in
        q)  QUERY="yes"
            ;;
        h)  usage_exit
            ;;
        \?) usage_exit
            ;;
    esac
done

shift $((OPTIND - 1))

if cat "$PROFILE" | grep -q "^COURSE=R\s*$"; then
    LR="right"
    echo -n "course: right"
else
    LR="left"
    echo -n "course: left"
fi

if [ "$QUERY" = "yes" ]; then
    echo ""
    exit 0
else
    PROFILE_BK="$PROFILE".bk
    cp "$PROFILE" "$PROFILE_BK"
    echo -n " -> "
    if [ "$LR" = "right" ]; then
	cat "$PROFILE_BK" | grep -v "^COURSE=R\s*$" > "$PROFILE"
	echo "COURSE=L" >> "$PROFILE"
	echo "left"
    else
	cat "$PROFILE_BK" | grep -v "^COURSE=L\s*$" > "$PROFILE"
	echo "COURSE=R" >> "$PROFILE"
	echo "right"
    fi
fi
