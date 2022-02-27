for i = 1:4
    a(i) = subplot(2,2,i);
    b(i) = annotation('textbox','String',"\theta_4",'Position',a(i).Position,'Vert','top','FitBoxToText','on', 'HorizontalAlignment', 'right')
end