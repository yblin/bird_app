#ifndef DIALOG_ADD_LABEL_H_
#define DIALOG_ADD_LABEL_H_

#include <QDialog>

namespace Ui {
class DialogAddLabel;
}

class DialogAddLabel : public QDialog {
    Q_OBJECT

public:
    explicit DialogAddLabel(QWidget *parent = nullptr);
    ~DialogAddLabel();

    QString label() const;
    void set_label(const QString& str);

private:
    Ui::DialogAddLabel* ui_;
};

#endif // DIALOG_ADD_LABEL_H_
