#include "dialog_add_label.h"
#include "ui_dialog_add_label.h"

DialogAddLabel::DialogAddLabel(QWidget *parent) :
    QDialog(parent),
    ui_(new Ui::DialogAddLabel) {
    ui_->setupUi(this);
}

DialogAddLabel::~DialogAddLabel() {
    delete ui_;
}

QString DialogAddLabel::label() const {
    return ui_->line_edit_label->text();
}

void DialogAddLabel::set_label(const QString& text) {
    ui_->line_edit_label->setText(text);
}
