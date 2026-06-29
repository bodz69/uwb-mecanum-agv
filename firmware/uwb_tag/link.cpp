#include "link.h"

//#define SERIAL_DEBUG

struct MyLink *init_link()
{
#ifdef SERIAL_DEBUG
    Serial.println("init_link");
#endif
    struct MyLink *p = (struct MyLink *)malloc(sizeof(struct MyLink));
    p->next = NULL;
    p->anchor_addr = 0;
    p->range[0] = 0.0;
    p->range[1] = 0.0;
    p->range[2] = 0.0;
    p->dbm = 0.0;
    return p;
}

void add_link(struct MyLink *p, uint16_t addr)
{
#ifdef SERIAL_DEBUG
    Serial.println("add_link");
#endif
    struct MyLink *temp = p;
    while (temp->next != NULL)
    {
        temp = temp->next;
    }

    struct MyLink *a = (struct MyLink *)malloc(sizeof(struct MyLink));
    a->anchor_addr = addr;
    a->range[0] = 0.0;
    a->range[1] = 0.0;
    a->range[2] = 0.0;
    a->dbm = 0.0;
    a->next = NULL;

    temp->next = a;
}

struct MyLink *find_link(struct MyLink *p, uint16_t addr)
{
#ifdef SERIAL_DEBUG
    Serial.println("find_link");
#endif
    if (addr == 0)
    {
        Serial.println("find_link:Input addr is 0");
        return NULL;
    }

    if (p->next == NULL)
    {
        Serial.println("find_link:Link is empty");
        return NULL;
    }

    struct MyLink *temp = p;
    while (temp->next != NULL)
    {
        temp = temp->next;
        if (temp->anchor_addr == addr)
        {
            return temp;
        }
    }

    Serial.println("find_link:Can't find addr");
    return NULL;
}

void fresh_link(struct MyLink *p, uint16_t addr, float range, float dbm)
{
#ifdef SERIAL_DEBUG
    Serial.println("fresh_link");
#endif
    struct MyLink *temp = find_link(p, addr);
    if (temp != NULL)
    {
        float prev_range = temp->range[0];
        float diff = fabs(prev_range - range);

        // 1. BỘ LỌC CHỐNG NHIỄU ĐỘT BIẾN (Outlier Rejection)
        const float OUTLIER_THRESHOLD = 1.5f;  // Mét
        if (prev_range > 0.01f && diff > OUTLIER_THRESHOLD)
        {
            Serial.print("Rejected outlier from anchor 0x");
            Serial.print(addr, HEX);
            Serial.print(": diff=");
            Serial.println(diff);
            return; // Bỏ qua giá trị nhiễu giật cục
        }

        // 2. BỘ LỌC THÔNG THẤP (LOW-PASS FILTER - LPF)
        // Hệ số Alpha quyết định độ mượt:
        // - Alpha càng NHỎ (vd: 0.1): Cực kỳ mượt, nhưng dữ liệu cập nhật bị trễ (lag).
        // - Alpha càng LỚN (vd: 0.8): Cập nhật nhanh, bám sát thực tế nhưng dễ bị rung.
        // Khuyến nghị cho xe chạy: 0.2 -> 0.3
        const float LPF_ALPHA = 0.25f; 

        float new_range;
        if (prev_range < 0.01f) {
            // Lần đầu tiên nhận giá trị, chưa có lịch sử để lọc
            new_range = range;
        } else {
            // Công thức Low-Pass Filter: Y_new = alpha * X_new + (1 - alpha) * Y_old
            new_range = (LPF_ALPHA * range) + ((1.0f - LPF_ALPHA) * prev_range);
        }

        // Cập nhật mảng lịch sử khoảng cách
        temp->range[2] = temp->range[1];
        temp->range[1] = temp->range[0];
        temp->range[0] = new_range;

        temp->dbm = dbm;
    }
    else
    {
        Serial.println("fresh_link:Fresh fail");
    }
}

void print_link(struct MyLink *p)
{
#ifdef SERIAL_DEBUG
    Serial.println("print_link");
#endif
    struct MyLink *temp = p;

    while (temp->next != NULL)
    {
        Serial.println(temp->next->anchor_addr, HEX);
        Serial.println(temp->next->range[0]);
        Serial.println(temp->next->dbm);
        temp = temp->next;
    }
}

void delete_link(struct MyLink *p, uint16_t addr)
{
#ifdef SERIAL_DEBUG
    Serial.println("delete_link");
#endif
    if (addr == 0) return;

    struct MyLink *temp = p;
    while (temp->next != NULL)
    {
        if (temp->next->anchor_addr == addr)
        {
            struct MyLink *del = temp->next;
            temp->next = del->next;
            free(del);
            return;
        }
        temp = temp->next;
    }
}

void make_link_json(struct MyLink *p, String *s)
{
#ifdef SERIAL_DEBUG
    Serial.println("make_link_json");
#endif
    *s = "{\"links\":[";
    struct MyLink *temp = p;

    while (temp->next != NULL)
    {
        temp = temp->next;
        char link_json[50];
        sprintf(link_json, "{\"A\":\"%X\",\"R\":\"%.2f\"}", temp->anchor_addr, temp->range[0]);
        *s += link_json;
        if (temp->next != NULL)
        {
            *s += ",";
        }
    }
    *s += "]}";
}