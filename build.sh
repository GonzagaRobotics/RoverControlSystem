# If the WebRTC package exists, link its NPM packages
if [ -d "/app/ros/src/web_rtc" ]; then
    (cd /app/ros/src/web_rtc && npm link puppeteer express ws)
fi

(cd /app/ros && colcon build --symlink-install)

source /app/ros/install/local_setup.bash