module mininav.viz.sink;

namespace mininav
{
    // 析构的 out-of-line 定义充当 VizSink 的 vtable "key function",把 vtable 与
    // typeinfo 锚定在 viz 库这一个翻译单元,避免跨 TU 的弱符号重复。
    VizSink::~VizSink() = default;
}
