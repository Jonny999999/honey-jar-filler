#include "app.h"

#include <stddef.h>

#include "app_params_def.h"

const app_param_meta_t *app_params_meta_get(size_t *out_count)
{
    static const app_param_meta_t meta[] = {
#define APP_PARAM_FLOAT(field, label_txt, unit_txt, def_val, min_val, max_val, step_val, brief_txt, detail_txt, group_txt) \
        {                                                                    \
            .name = #field,                                                  \
            .label = label_txt,                                              \
            .unit = unit_txt,                                                \
            .desc_brief = brief_txt,                                         \
            .desc_detail = detail_txt,                                       \
            .group = group_txt,                                              \
            .type = APP_PARAM_FLOAT,                                         \
            .offset = offsetof(app_params_t, field),                         \
            .def.f = (def_val),                                              \
            .min.f = (min_val),                                              \
            .max.f = (max_val),                                              \
            .step.f = (step_val),                                            \
        },
#define APP_PARAM_U32(field, label_txt, unit_txt, def_val, min_val, max_val, step_val, brief_txt, detail_txt, group_txt) \
        {                                                                    \
            .name = #field,                                                  \
            .label = label_txt,                                              \
            .unit = unit_txt,                                                \
            .desc_brief = brief_txt,                                         \
            .desc_detail = detail_txt,                                       \
            .group = group_txt,                                              \
            .type = APP_PARAM_U32,                                           \
            .offset = offsetof(app_params_t, field),                         \
            .def.u32 = (def_val),                                            \
            .min.u32 = (min_val),                                            \
            .max.u32 = (max_val),                                            \
            .step.u32 = (step_val),                                          \
        },
#define APP_PARAM_U8(field, label_txt, unit_txt, def_val, min_val, max_val, step_val, brief_txt, detail_txt, group_txt) \
        {                                                                    \
            .name = #field,                                                  \
            .label = label_txt,                                              \
            .unit = unit_txt,                                                \
            .desc_brief = brief_txt,                                         \
            .desc_detail = detail_txt,                                       \
            .group = group_txt,                                              \
            .type = APP_PARAM_U8,                                            \
            .offset = offsetof(app_params_t, field),                         \
            .def.u8 = (def_val),                                             \
            .min.u8 = (min_val),                                             \
            .max.u8 = (max_val),                                             \
            .step.u8 = (step_val),                                           \
        },
        APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8)
#undef APP_PARAM_FLOAT
#undef APP_PARAM_U32
#undef APP_PARAM_U8
    };

    if (out_count) {
        *out_count = sizeof(meta) / sizeof(meta[0]);
    }
    return meta;
}
