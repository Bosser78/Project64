SET(SOURCES screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_eight_32_png.c
    images/ui_img_eight_64_png.c
    images/ui_img_five_64_png.c
    images/ui_img_four_64_png.c
    images/ui_img_nine_64_png.c
    images/ui_img_one_64_png.c
    images/ui_img_pause_32_png.c
    images/ui_img_seven_64_png.c
    images/ui_img_six_64_png.c
    images/ui_img_three_64_png.c
    images/ui_img_two_64_png.c
    images/ui_img_zero_64_png.c
    fonts/ui_font_Font2.c)

add_library(ui ${SOURCES})
