function logger(executing_file, message)
    logger_message = append('Logger: ', executing_file, '.m', ' - ', message);
    disp(logger_message);
end