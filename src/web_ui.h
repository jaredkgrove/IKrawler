#pragma once

/**
 * Web UI for hexapod controller — embedded from src/web_ui.html at build time.
 *
 * Edit the .html file directly (full syntax highlighting, Emmet, etc.).
 * PlatformIO's board_build.embed_files links it into flash automatically.
 */

extern const char web_ui_html_start[] asm("_binary_src_web_ui_html_start");
