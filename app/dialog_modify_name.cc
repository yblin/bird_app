#include "dialog_modify_name.h"
#include "ui_dialog_modify_name.h"

DialogModifyName::DialogModifyName(QWidget *parent) :
    QDialog(parent),
    ui_(new Ui::DialogModifyName) {
    ui_->setupUi(this);
}

DialogModifyName::~DialogModifyName() {
    delete ui_;
}

QString DialogModifyName::original_scientific_name() const {
    return ui_->line_edit_original_name->text();
}

QString DialogModifyName::modified_scientific_name() const {
    return ui_->line_edit_modified_name->text();
}

void DialogModifyName::set_original_scientific_name(
        const std::string& name) {
    ui_->line_edit_original_name->setText(QString::fromStdString(name));
}
