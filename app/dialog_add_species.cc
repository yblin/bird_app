#include "dialog_add_species.h"
#include "ui_dialog_add_species.h"

DialogAddSpecies::DialogAddSpecies(QWidget *parent) :
    QDialog(parent),
    ui_(new Ui::DialogAddSpecies) {
    ui_->setupUi(this);
}

DialogAddSpecies::~DialogAddSpecies() {
    delete ui_;
}

QString DialogAddSpecies::scientific_name() const {
    return ui_->line_edit_scientific_name->text();
}

QString DialogAddSpecies::english_name() const {
    return ui_->line_edit_english_name->text();
}

QString DialogAddSpecies::chinese_name() const {
    return ui_->line_edit_chinese_name->text();
}

void DialogAddSpecies::set_scientific_name(const QString& str) {
    ui_->line_edit_scientific_name->setText(str);
}

void DialogAddSpecies::set_english_name(const QString& str) {
    ui_->line_edit_english_name->setText(str);
}

void DialogAddSpecies::set_chinese_name(const QString& str) {
    ui_->line_edit_chinese_name->setText(str);
}
