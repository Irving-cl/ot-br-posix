#!/bin/bash
#

OTBR_DBUS_SERVER_CONF=otbr-test-agent.conf
readonly OTBR_DBUS_SERVER_CONF
PROJECT_SOURCE_DIR=/usr/local/google/home/irvingcl/Documents2/github/ot-br-posix
CMAKE_BINARY_DIR=/usr/local/google/home/irvingcl/Documents2/github/ot-br-posix/build/otbr
readonly CMAKE_BINARY_DIR
OT_NCP_PATH=/usr/local/google/home/irvingcl/Documents/git/openthread/build/simulation/examples/apps/ncp/ot-ncp-ftd

on_exit()
{
    local status=$?

    sudo systemctl stop test-otbr-agent || true
    sudo killall dbus-monitor || true
    sudo rm "/etc/dbus-1/system.d/${OTBR_DBUS_SERVER_CONF}" || true

    return "${status}"
}

ot_ctl()
{
    CMAKE_BINARY_DIR=/usr/local/google/home/irvingcl/Documents2/github/ot-br-posix/build/otbr
    sudo "${CMAKE_BINARY_DIR}"/third_party/openthread/repo/src/posix/ot-ctl "$@"
}

suite_setup()
{
    TEST_HELLO="$(basename "$0") started at $(date +%s)"
    logger "$TEST_HELLO"
    [[ -f /etc/dbus-1/system.d/"${OTBR_DBUS_SERVER_CONF}" ]] || {
        local CONFIG_FILE_PATH="third_party/openthread/repo/src/posix/platform"
        mkdir -p "${PWD}/${CONFIG_FILE_PATH}" && cp "${PROJECT_SOURCE_DIR}/${CONFIG_FILE_PATH}/openthread.conf.example" "${PWD}/${CONFIG_FILE_PATH}"

        sudo rm -rf tmp
        sudo install -m 644 "${CMAKE_BINARY_DIR}"/src/agent/otbr-agent.conf /etc/dbus-1/system.d/"${OTBR_DBUS_SERVER_CONF}"
        sudo service dbus reload
    }
    trap on_exit EXIT

    sudo systemctl start avahi-daemon

    export -f ot_ctl
}

test_ready_signal()
{
    # Because we do want to run the command as root but redirect as the normal user.
    # shellcheck disable=SC2024
    sudo expect <<EOF
spawn dbus-monitor --system path=/io/openthread/BorderRouter/wpan0,member=Ready
set dbus_monitor \$spawn_id
spawn ${CMAKE_BINARY_DIR}/src/agent/otbr-agent -d7 -I wpan0 spinel+hdlc+forkpty://${OT_NCP_PATH}?forkpty-arg=1&skip-rcp-compatibility-check=1
set spawn_id \$dbus_monitor
expect {
    "member=Ready" { exit }
    timeout { error "Failed to find Ready signal\n" }
    echo "Hello"
}
EOF
    pidof ot-ncp-ftd
    timeout 2 bash -c "while pidof ot-ncp-ftd; do logger -t otbr-test-dbus-client; sleep 1; done"
}

otbr_agent_service_start()
{
    local -r EXIT_CODE_SHOULD_RESTART=7

    sudo systemd-run --collect --no-ask-password -u test-otbr-agent -p "RestartForceExitStatus=$EXIT_CODE_SHOULD_RESTART" "${CMAKE_BINARY_DIR}"/src/agent/otbr-agent -d7 -I wpan0 -B enxf8e43b90fb62 "spinel+hdlc+forkpty://${OT_NCP_PATH}?forkpty-arg=1&skip-rcp-compatibility-check=1"
    # Mark: offload command
    timeout 2 bash -c "while ! ot_ctl offload state; do sleep 1; done"
}

main()
{
    suite_setup

    test_ready_signal

    otbr_agent_service_start

    sleep 10000
}

main "$@"
