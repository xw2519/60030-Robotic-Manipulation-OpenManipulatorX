function logger(executing_file, message)
    logger_message = append('[Log]: ', executing_file, '.m', ' - ', message);
    disp(logger_message);
end